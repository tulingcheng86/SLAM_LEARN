echo "Start a Tmux session!"

# 创建一个名为 "ego_planner" 的 Tmux 会话
tmux new -d -s ego_planner

echo "Start ego_planner_1!"

# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav.launch" ENTER 

sleep 22

echo "Start Rotating!"
tmux send-keys -t ego_planner "C-c" ENTER
sleep 2
tmux send-keys -t ego_planner "python xuanzhuan.py" ENTER 
sleep 6

echo "Start ego_planner_2!"
# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav_2.launch" ENTER 
sleep 15

echo "Start Rotating!"
tmux send-keys -t ego_planner "C-c" ENTER
sleep 2
tmux send-keys -t ego_planner "python xuanzhuan.py" ENTER 
sleep 6

echo "Start GOhome!!"
# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav_3.launch" ENTER 



