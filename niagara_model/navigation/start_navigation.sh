#!/bin/bash

# Navigation2 阿克曼车辆启动脚本
# 使用方法: ./start_navigation.sh [选项]

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")")"

# 默认参数
CHECK_SYSTEM=true
START_MAP_SERVER=false
START_RVIZ=false
MAP_FILE=""
PARAMS_FILE="$SCRIPT_DIR/config/nav2_params.yaml"
BT_FILE="$SCRIPT_DIR/behavior_trees/ackermann_nav_bt.xml"

# 帮助信息
show_help() {
    echo -e "${BLUE}Navigation2 阿克曼车辆启动脚本${NC}"
    echo ""
    echo "使用方法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -c, --check             运行系统检查（默认启用）"
    echo "  --no-check              跳过系统检查"
    echo "  -m, --map FILE          启动地图服务器并加载指定地图文件"
    echo "  -r, --rviz              启动RViz可视化"
    echo "  -p, --params FILE       指定参数文件路径"
    echo "  -b, --bt FILE           指定行为树文件路径"
    echo ""
    echo "示例:"
    echo "  $0                      # 基本启动（仅Navigation2）"
    echo "  $0 -c -r                # 启动并运行检查和RViz"
    echo "  $0 -m /path/to/map.yaml # 启动地图服务器和Navigation2"
    echo "  $0 --no-check -r        # 跳过检查，启动RViz"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--check)
            CHECK_SYSTEM=true
            shift
            ;;
        --no-check)
            CHECK_SYSTEM=false
            shift
            ;;
        -m|--map)
            START_MAP_SERVER=true
            MAP_FILE="$2"
            shift 2
            ;;
        -r|--rviz)
            START_RVIZ=true
            shift
            ;;
        -p|--params)
            PARAMS_FILE="$2"
            shift 2
            ;;
        -b|--bt)
            BT_FILE="$2"
            shift 2
            ;;
        *)
            echo -e "${RED}错误: 未知选项 $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 打印启动信息
echo -e "${BLUE}=== Navigation2 阿克曼车辆启动脚本 ===${NC}"
echo -e "工作空间: ${WORKSPACE_DIR}"
echo -e "参数文件: ${PARAMS_FILE}"
echo -e "行为树文件: ${BT_FILE}"
echo ""

# 检查工作空间
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}错误: 工作空间目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

# 进入工作空间
cd "$WORKSPACE_DIR"

# 检查是否已编译
if [ ! -d "install" ]; then
    echo -e "${YELLOW}警告: 未找到install目录，正在编译工作空间...${NC}"
    colcon build --packages-select niagara_model
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误: 编译失败${NC}"
        exit 1
    fi
fi

# 设置环境
echo -e "${BLUE}设置ROS2环境...${NC}"
source install/setup.bash

# 运行系统检查
if [ "$CHECK_SYSTEM" = true ]; then
    echo -e "${BLUE}运行系统检查...${NC}"
    python3 "$SCRIPT_DIR/nav2_ackermann_checker.py"
    
    echo -e "${YELLOW}检查完成。是否继续启动Navigation2? (y/N)${NC}"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}启动已取消${NC}"
        exit 0
    fi
fi

# 检查参数文件
if [ ! -f "$PARAMS_FILE" ]; then
    echo -e "${RED}错误: 参数文件不存在: $PARAMS_FILE${NC}"
    exit 1
fi

# 检查行为树文件
if [ ! -f "$BT_FILE" ]; then
    echo -e "${RED}错误: 行为树文件不存在: $BT_FILE${NC}"
    exit 1
fi

# 启动地图服务器（如果需要）
if [ "$START_MAP_SERVER" = true ]; then
    if [ -z "$MAP_FILE" ] || [ ! -f "$MAP_FILE" ]; then
        echo -e "${RED}错误: 地图文件不存在或未指定: $MAP_FILE${NC}"
        exit 1
    fi
    
    echo -e "${BLUE}启动地图服务器...${NC}"
    gnome-terminal --title="Map Server" -- bash -c "
        source install/setup.bash
        ros2 launch nav2_bringup bringup_launch.py map:='$MAP_FILE'
        exec bash
    " &
    
    # 等待地图服务器启动
    echo -e "${YELLOW}等待地图服务器启动...${NC}"
    sleep 3
fi

# 启动Navigation2
echo -e "${BLUE}启动Navigation2...${NC}"
gnome-terminal --title="Navigation2" -- bash -c "
    source install/setup.bash
    ros2 launch niagara_model nav2_bringup.launch.py \
        params_file:='$PARAMS_FILE' \
        default_nav_to_pose_bt_xml:='$BT_FILE'
    exec bash
" &

# 等待Navigation2启动
echo -e "${YELLOW}等待Navigation2启动...${NC}"
sleep 5

# 启动RViz（如果需要）
if [ "$START_RVIZ" = true ]; then
    echo -e "${BLUE}启动RViz...${NC}"
    gnome-terminal --title="RViz" -- bash -c "
        source install/setup.bash
        ros2 launch nav2_bringup rviz_launch.py
        exec bash
    " &
fi

echo -e "${GREEN}=== 启动完成 ===${NC}"
echo -e "${GREEN}Navigation2已启动，您可以：${NC}"
echo -e "1. 在RViz中设置初始位姿（2D Pose Estimate）"
echo -e "2. 设置导航目标（2D Nav Goal）"
echo -e "3. 监控导航状态和日志"
echo ""
echo -e "${YELLOW}提示: 使用Ctrl+C停止此脚本不会关闭已启动的节点${NC}"
echo -e "${YELLOW}要停止所有节点，请关闭相应的终端窗口${NC}"
echo ""

# 保持脚本运行，监控状态
echo -e "${BLUE}监控Navigation2状态...（按Ctrl+C退出监控）${NC}"
while true; do
    sleep 10
    # 检查关键节点是否还在运行
    if ! ros2 node list | grep -q "bt_navigator"; then
        echo -e "${RED}警告: bt_navigator节点已停止${NC}"
    fi
    if ! ros2 node list | grep -q "controller_server"; then
        echo -e "${RED}警告: controller_server节点已停止${NC}"
    fi
done