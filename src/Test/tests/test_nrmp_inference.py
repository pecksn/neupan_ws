"""Production NRMP/full PAN against captured failures; no ROS or vehicle I/O."""
from pathlib import Path
import unittest
import time
import json
from unittest.mock import patch
import numpy as np
import torch
from neupan.robot import robot
from neupan.blocks.nrmp import NRMP
from neupan.blocks.pan import PAN

ROOT = Path(__file__).resolve().parents[3]


class NRMPInferenceTests(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1)
        self.bot = robot(receding=50, step_time=.1, kinematics='acker', wheelbase=.67,
                         max_speed=[1.5,.35], max_acce=[.8,.08], length=.95, width=.85)
        self.paths = sorted((ROOT/'tracker_logs').glob('neupan_failure_20261109_*.npz'))
        if not self.paths:
            self.skipTest('captured Nov9 NRMP failures required')

    def test_both_exact_failure_parameters_solve_in_same_cached_problem(self):
        layer = NRMP(50,.1,self.bot,nrmp_max_num=10, inference_solver='CLARABEL').eval()
        identity = id(layer.problem)
        for path in self.paths:
            arrays = np.load(str(path),allow_pickle=False)
            keys = sorted((k for k in arrays.files if '_parameter' in k),key=lambda k:int(k.split('_parameter')[1]))
            values = [torch.from_numpy(arrays[k]) for k in keys]
            result = layer.solve_inference(values)
            self.assertEqual(id(layer.problem), identity)
            self.assertEqual([tuple(t.shape) for t in result], [(3,51),(2,50),(1,50)])
            self.assertTrue(all(torch.isfinite(t).all() for t in result))
            self.assertLessEqual(layer.solve_diagnostics['max_hard_constraint_violation'], 1.e-6)
            print('NRMP_CAPTURE '+json.dumps(dict(snapshot=path.name,**layer.solve_diagnostics)),flush=True)
            broken = [t.clone() for t in values]; broken[0][0,0] = float('nan')
            with self.assertRaises(ValueError): layer.solve_inference(broken)
            infeasible=[t.clone() for t in values];infeasible[-1]=infeasible[-2]+1.
            with self.assertRaises(RuntimeError):layer.solve_inference(infeasible)
            layer.train()
            with patch.object(layer,'generate_parameter_value',return_value=values), patch.object(
                    layer.nrmp_layer,'forward',side_effect=RuntimeError('original differentiable path')):
                with self.assertRaisesRegex(RuntimeError,'original differentiable path'):
                    layer(None,None,None,None)
            layer.eval()

    def test_full_pan_including_dune_and_both_iterations(self):
        for path in self.paths:
            metadata=json.loads(path.with_suffix('.json').read_text())
            checkpoint=ROOT/'src/neupan_ros/example/lxw_car/model'/Path(metadata['dune_checkpoint']).parent.name/'model_5000.pth'
            pan = PAN(50,.1,self.bot,iter_num=2,dune_max_num=100,nrmp_max_num=10,
                dune_checkpoint=str(checkpoint),adjust_kwargs=dict(inference_solver='CLARABEL',
                    q_s=1.2,p_u=21.,eta=1.,d_max=.8,d_min=.2))
            arrays = np.load(str(path),allow_pickle=False)
            values = [torch.from_numpy(arrays['frame7_'+k]).float() for k in ('nom_s','nom_u','ref_s','ref_us')]
            iterations=[]
            hook=pan.nrmp_layer.register_forward_hook(lambda *args:iterations.append(1))
            began=time.monotonic()
            result = pan(*values,torch.from_numpy(arrays['solver_obstacle_points']).float())
            elapsed=time.monotonic()-began
            hook.remove()
            self.assertEqual(len(iterations),2)
            self.assertTrue(all(torch.isfinite(t).all() for t in result))
            controls = result[1].detach().cpu().numpy()
            self.assertLessEqual(float(np.max(abs(controls[0]))),1.5+1.e-5)
            self.assertLessEqual(float(np.max(abs(controls[1]))),.35+1.e-5)
            self.assertLessEqual(float(np.max(abs(np.diff(controls[1])))),.008+1.e-5)
            self.assertLess(elapsed,.5,'full PAN exceeded existing control deadline')
            self.assertLessEqual(pan.nrmp_layer.solve_diagnostics['returned_tensor_residual'],1.e-4)
            print('FULL_PAN_CAPTURE '+json.dumps(dict(snapshot=path.name,checkpoint=str(checkpoint),iterations=len(iterations),elapsed_sec=elapsed,
                first_native_action=controls[:,0].tolist(),nrmp=pan.nrmp_layer.solve_diagnostics)),flush=True)

    def test_explicit_training_retains_actual_cvxpy_gradient(self):
        layer=NRMP(50,.1,self.bot,nrmp_max_num=0,inference_solver='CLARABEL').train()
        nominal=torch.zeros((3,51));controls=torch.zeros((2,50))
        reference=torch.zeros((3,51));reference[0]=torch.linspace(0.,.2,51)
        reference.requires_grad_(True)
        state,velocity,_=layer(nominal,controls,reference,torch.full((50,),.04))
        (state.square().sum()+velocity.square().sum()).backward()
        self.assertIsNotNone(reference.grad)
        self.assertTrue(torch.isfinite(reference.grad).all())
        self.assertEqual(layer.solve_diagnostics,{})  # Never entered direct inference.

