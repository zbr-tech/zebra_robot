https://github.com/Derek-TH-Wang/quadruped_ctrl  
# Memo
## Overall
- main文はscripts/walking_simulation.pyにある。このうちrun()関数が500Hzでループされる処理。runで実行される目標関節トルクtauを求める関数cpp_gait_ctrller.toque_calculatorが解読すべきプログラム
これはscr/GaitCtrllerで定義されている。
- シミュレーションはマイコン部の代わりでしかないので実機では不要。
- MPC系はcppのプログラムを共有ライブラリ化して使っているのでGaitCtrller.hを読む必要がある。
- パラメータは基本クラス定義部分で直打ちしてある（修正すべき？）
MPC_Ctrl/Gait.h/OffsetDurationGait：歩容クラス
MPC_Ctrl/ConvexMPCLocomotion.cppでインスタンスを作成している。defaultではtrot。initSparseMPC()でmassパラメータに値を入れている
- 足先座標はab/ad原点で、座標軸は機体と同じ（前x,左ｙ上z)
姿勢推定クラス
- VectorNavOrientationEstimator：IMUを用いた姿勢推定
- LinearKFPositionVelocityEstimator：カルマンフィルタ.  PositionVelocityEstimator.cppの20行目にパラメータdtがある．
- StateEstimate：推定結果 rBody*world=機体座標系  
ハードに関するパラメータ：
- Dynamics/MiniCheetah.h
- /home/usr/.local/lib/python2.7/site-packages/pybullet_data/mini_cheetah/mini_cheetah.urdf
足先軌道：
- Control/FootSwingTrajectoryを使って計算
- 足上げ高さのパラメータ：MPC_Ctrl/ConvexMPCLocomotion.cppの290行目
- MPCに関するパラメータ
    -  DesiredStateCommand : 使用されていない
- 安全チェックに関するパラメータ
    - src/Controllers/SafetyChecker.cpp に直書き


### Others
- solveMPC.cpp/q_soln：目標反力（world座標系）
- ConvexMPCLocomotion.cpp/solveDenseMPC() で目標反力を取得している
- jcqpは使用しない(ConvexMPCLocomotion cpp 650行目)
- wbcは使用しない 
