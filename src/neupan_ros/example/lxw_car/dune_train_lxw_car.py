from neupan import neupan

if __name__ == '__main__':
    # 加载你自己的配置文件 (使用专用训练配置)
    neupan_planner = neupan.init_from_yaml('config/neupan_train.yaml')
    # 开始训练 DUNE 模型
    neupan_planner.train_dune()
