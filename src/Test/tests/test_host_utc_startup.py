"""Container startup refreshes independent host evidence without opening devices."""
import ast
import json
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver import host_utc_evidence as host


class HostStartupTests(unittest.TestCase):
    def test_full_main_publishes_wait_then_ready_before_one_serial_open(self):
        from libnmea_navsat_driver.serial_diagnostics import SerialDiagnostics,BoundedRawCapture
        source=ROOT/'src/wheeltec_gps_driver/src/libnmea_navsat_driver/nodes/nmea_serial_driver.py'
        functions=[n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)]
        ros=Mock();ros.get_param.side_effect=lambda key,default=None: {
            '~gnss_common_utc_verified':True,'~host_clock_evidence_file':'test.json'}.get(key,default)
        ros.is_shutdown.side_effect=[False,False,True]
        ros.Time.now.return_value.to_sec.return_value=1.
        ros.get_name.return_value='nmea_test'
        driver=Mock();driver.observation_timing.summary.return_value={}
        driver_class=Mock(return_value=driver)
        driver_class.get_frame_id.return_value='gps'
        namespace=dict(rospy=ros,RosNMEADriver=driver_class,
            refresh_host_utc=Mock(side_effect=[{'qualified':False,'error':'missing'}, {'qualified':True}]),
            SerialDiagnostics=SerialDiagnostics,BoundedRawCapture=BoundedRawCapture,
            source_identity=lambda path:{'path':path},__file__=str(source),
            driver_module=NS(__file__='driver.py'),unicore=NS(__file__='unicore.py'),
            observation_time=NS(__file__='timing.py',host_clock_evidence=lambda path:{}),
            time=__import__('time'),os=__import__('os'),sys=sys,json=json,
            String=lambda **kw:NS(**kw))
        def serial_open(**kwargs):
            reports=[json.loads(call.args[0].data) for call in ros.Publisher.return_value.publish.call_args_list]
            self.assertEqual([v['state'] for v in reports],['waiting_host_utc_evidence','host_utc_ready'])
            return Mock()
        namespace['serial']=NS(Serial=Mock(side_effect=serial_open),SerialException=OSError)
        ros.ROSInterruptException=InterruptedError
        exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),namespace)
        namespace['main']()
        self.assertEqual(namespace['serial'].Serial.call_count,1)
        ros.Timer.return_value.shutdown.assert_called_once()

    def evidence(self, age=0.):
        return dict(qualified=True, utc_ns=1000000000000-int(age*1e9),
                    monotonic_ns=500000000000-int(age*1e9), boot_id='boot',
                    sample_age_sec=348., max_sample_age_sec=1084.)

    def test_each_start_refreshes_even_when_previous_capture_expired(self):
        read_text=Path.read_text
        recorded=json.loads((ROOT/'tracker_logs/review_20261111/morning_diagnosis/system_run_full_20261111_094053_serial.json').read_text())[0]['host_clock_evidence']['host_snapshot']
        for age in (None,57.7,226.,287.2):
            with self.subTest(age=age), tempfile.TemporaryDirectory() as directory:
                target=Path(directory)/'host.json'
                if age is not None:target.write_text(json.dumps(recorded))
                age=age or 0.
                utc_ns=recorded['utc_ns']+round(age*1e9)
                mono_ns=recorded['monotonic_ns']+round(age*1e9)
                fresh=dict(recorded,utc_ns=utc_ns,monotonic_ns=mono_ns,
                           sample_age_sec=recorded['sample_age_sec']+age)
                with patch.object(host, 'query', return_value=json.dumps(fresh)) as query, \
                     patch.object(host.time, 'time_ns', return_value=utc_ns), \
                     patch.object(host.time, 'monotonic_ns', return_value=mono_ns), \
                     patch.object(host.Path, 'read_text', autospec=True,
                         side_effect=lambda p, **kw: recorded['boot_id'] if str(p).startswith('/proc/') else read_text(p,**kw)):
                    self.assertEqual(host.validate(recorded),age<=120.)
                    result=host.refresh(target)
                    self.assertTrue(result['qualified'], result)
                    self.assertTrue(host.validate(json.loads(target.read_text())))
                    self.assertEqual(query.call_count, 1)
                    args=query.call_args.args
                    self.assertEqual(args[0], 'ssh')
                    self.assertIn('nuc15@127.0.0.1', args)
                    self.assertIn('StrictHostKeyChecking=yes', args)

    def test_bad_host_evidence_cannot_be_replaced_by_old_good_file(self):
        read_text=Path.read_text
        for bad in (dict(qualified=False, error='NTP unavailable'),
                    dict(self.evidence(), boot_id='other'),
                    dict(self.evidence(), sample_age_sec=2000.),
                    dict(self.evidence(), utc_ns=990000000000)):
            with tempfile.TemporaryDirectory() as directory, \
                 patch.object(host,'query',return_value=json.dumps(bad)), \
                 patch.object(host.time,'time_ns',return_value=1000000000000), \
                 patch.object(host.time,'monotonic_ns',return_value=500000000000), \
                 patch.object(host.Path,'read_text',autospec=True,
                     side_effect=lambda p, **kw:'boot' if str(p).startswith('/proc/') else read_text(p,**kw)):
                target=Path(directory)/'host.json'
                target.write_text(json.dumps(self.evidence()))
                self.assertTrue(host.validate(self.evidence()))
                result=host.refresh(target)
                self.assertFalse(result['qualified'])
                self.assertIn('error', result)
                self.assertFalse(host.validate(json.loads(target.read_text())))

    def test_unavailable_channel_is_structured_not_an_exception(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(host,'query',side_effect=OSError('connection refused')):
            result=host.refresh(Path(directory)/'host.json')
            self.assertFalse(result['qualified'])
            self.assertIn('connection refused', result['error'])

    def test_actual_serial_entry_waits_with_diagnostics_before_device_open(self):
        source=ROOT/'src/wheeltec_gps_driver/src/libnmea_navsat_driver/nodes/nmea_serial_driver.py'
        tree=ast.parse(source.read_text())
        function=next(v for v in tree.body if isinstance(v,ast.FunctionDef) and v.name=='wait_for_host_utc')
        namespace={'rospy':Mock(), 'refresh_host_utc':Mock(side_effect=[
            {'qualified':False,'error':'host unavailable'}, {'qualified':True}])}
        namespace['rospy'].is_shutdown.return_value=False
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),namespace)
        diagnostics=Mock(); publish=Mock()
        self.assertTrue(namespace['wait_for_host_utc']('host.json',diagnostics,publish))
        self.assertEqual(publish.call_count, 2)
        diagnostics.set_state.assert_any_call('waiting_host_utc_evidence')
        self.assertEqual(namespace['rospy'].sleep.call_count, 1)
        main=next(v for v in tree.body if isinstance(v,ast.FunctionDef) and v.name=='main')
        calls=[v.func.id for v in ast.walk(main) if isinstance(v,ast.Call) and isinstance(v.func,ast.Name)]
        self.assertIn('wait_for_host_utc', calls)


if __name__=='__main__': unittest.main()
