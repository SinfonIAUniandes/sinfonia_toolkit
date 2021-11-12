#!/bin/sh

if [ "$1" = "-h" ] || [ "$1" = "--help" ]
	then :
		echo "Arguments:"
		echo "  -m, --mode=MODE        Defines the execution mode"
		echo "                         --------------------------------------------------"
		echo "                         --------------------------------------------------"
		echo "                         MODE = compile, COMPILE, c, C"
		echo "                         Compiles the robot_toolkit files inside the VM"
		echo "                         REQUIRES: --vnip and --vnpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = install, INSTALL, i, I"
		echo "                         Compiles the robot_toolkit files inside the VM and"
		echo "                         installs the resulting binary files inside the robot"
		echo "                         REQUIRES: --vnip, --robotip, --vnpass and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = run, RUN, r, R"
		echo "                         Runs the robot_toolkit on the Robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = all, ALL, a, A"
		echo "                         Compiles the robot_toolkit files inside the VM and"
		echo "                         installs the resulting binary files inside the robot"
		echo "                         and finally runs the toolkit on the robot"
		echo "                         REQUIRES: --vnip, --robotip, --vnpass and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = kill, KILL, k, K"
		echo "                         Kills robot_toolkit procceses inside the robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_vn, CLEAN_VN, cv, CV"
		echo "                         Ereases the compilation and binary files inside the"
		echo "                         virtual nao"
		echo "                         REQUIRES: --vnip and --vnpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_robot, CLEAN_ROBOT, cr, CR"
		echo "                         Ereases the binary files inside the robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_all, CLEAN_ALL, ca, CA"
		echo "                         Ereases the compilation and binary files inside the"
		echo "                         virtual nao and also the binary files inside"
		echo "                         the robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "  -v, --vnip=IP          IP = Virtual Nao IP"
		echo "  -r, --robotip=IP       IP = Robot IP"
		echo "  -n, --vnpass=PASSWORD  PASSWORD = Virtual Nao password"
		echo "  -p, --robotpass=IP     PASSWORD = Robot password"



	exit
fi

for i in "$@"
do
case $i in
    -m=*|--mode=*)
    MODE="${i#*=}"
    shift # past argument=value
    ;;
    -r=*|--robotip=*)
    ROBOTIP="${i#*=}"
    shift # past argument=value
    ;;
    -v=*|--vnip=*)
    VNIP="${i#*=}"
    shift # past argument=value
    ;;
    -n=*|--vnpass=*)
    VNPASS="${i#*=}"
    shift # past argument=value
    ;;
    -p=*|--robotpass=*)
    ROBOTPASS="${i#*=}"
    shift # past argument=value
    ;;
    --default)
    DEFAULT=YES
    shift # past argument with no value
    ;;
    *)
          # unknown option
    ;;
esac
done

if [ -z ${MODE} ]
	then echo "Not enough arguments"
	exit
fi
if [ ${MODE} = "compile" ] || [ ${MODE} = "COMPILE" ] || [ ${MODE} = "c" ] || [ ${MODE} = "C" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	MODE="compile"
fi
if [ ${MODE} = "install" ] || [ ${MODE} = "INSTALL" ] || [ ${MODE} = "i" ] || [ ${MODE} = "I" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments, robot IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="install"
fi
if [ ${MODE} = "run" ] || [ ${MODE} = "RUN" ] || [ ${MODE} = "r" ] || [ ${MODE} = "R" ]
	then :
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="run"
fi
if [ ${MODE} = "all" ] || [ ${MODE} = "ALL" ] || [ ${MODE} = "a" ] || [ ${MODE} = "A" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments, robot IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="all"
fi
if [ ${MODE} = "kill" ] || [ ${MODE} = "KILL" ] || [ ${MODE} = "k" ] || [ ${MODE} = "K" ]
	then :
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="kill"
fi
if [ ${MODE} = "clean_vn" ] || [ ${MODE} = "CLEAN_VN" ] || [ ${MODE} = "cv" ] || [ ${MODE} = "CV" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	MODE="clean_vn"
fi
if [ ${MODE} = "clean_robot" ] || [ ${MODE} = "CLEAN_ROBOT" ] || [ ${MODE} = "cr" ] || [ ${MODE} = "CR" ]
	then :
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="clean_robot"
fi
if [ ${MODE} = "clean_all" ] || [ ${MODE} = "CLEAN_ALL" ] || [ ${MODE} = "ca" ] || [ ${MODE} = "CA" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments, robot IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="clean_all"
fi
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'


#echo "======================="
#echo "MODE      = ${MODE}"
#echo "ROBOTIP   = ${ROBOTIP}"
#echo "VNIP      = ${VNIP}"
#echo "VNPASS    = ${VNPASS}"
#echo "ROBOTPASS = ${ROBOTPASS}"
#echo "======================="

clean_vn()
{
	echo "${YELLOW}Cleaning virtual nao... ${NC}"
	COMMAND="sudo rm -rf /home/nao/.temp/ && sudo rm -rf /home/nao/build_isolated/robot_toolkit && sudo rm -rf /home/nao/devel_isolated/robot_toolkit && sudo rm -rf /home/nao/ros_toolchain_install/lib/robot_toolkit && sudo rm -rf /home/nao/ros_toolchain_install/lib/pkgconfig/robot_toolkit.pc && sudo rm -rf /home/nao/ros_toolchain_install/share/robot_toolkit && sudo rm -rf /home/nao/ros_toolchain_install/lib/librobot_toolkit_module.so && sudo rm -rf /home/nao/ros_toolchain_install/lib/librobot_toolkit.so && sudo rm -rf /home/nao/src/robot_toolkit && sudo rm -rf /home/nao/ros_toolchain_install/share/qi/module/robot_toolkit_module.mod"
	sshpass -p ${VNPASS} ssh nao@${VNIP} ${COMMAND}
	echo "Executing: ${COMMAND}"
	echo "${GREEN}virtual nao cleaned successfully ${NC}"
}

clean_robot()
{
	echo "${YELLOW}Cleaning robot... ${NC}"	
	COMMAND="rm -rf /home/nao/ros/lib/robot_toolkit && rm -rf /home/nao/ros/lib/pkgconfig/robot_toolkit.pc && rm -rf /home/nao/ros/share/robot_toolkit && rm -rf /home/nao/ros/lib/librobot_toolkit_module.so && rm -rf /home/nao/ros/lib/librobot_toolkit.so && rm -rf /home/nao/ros/share/qi/module/robot_toolkit_module.mod"
	sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} ${COMMAND}
	echo "Executing: ${COMMAND}"
	echo "${GREEN}robot cleaned successfully ${NC}"
}

kill_process()
{
	COMMAND="source /home/nao/ros/setup.bash && rosnode kill -a && killall -9 rosmaster && killall robot_toolkit_node"
	sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} ${COMMAND}
	echo ${COMMAND}
	echo "${GREEN}robot_toolkit procceses killed successfully${NC}"
}

if [ ${MODE} = "compile" ] || [ ${MODE} = "all" ]
	then :
		echo "${YELLOW}Starting compilation ... ${NC}"
		OUTPUT="$(sshpass -p ${VNPASS} ssh nao@${VNIP} 'pwd')"
		echo ${OUTPUT}
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Compilation failed, please check the password  and IP ${NC}"
				exit
		fi
		clean_vn
		sshpass -p ${VNPASS} scp -r ../robot_toolkit nao@${VNIP}:/home/nao/src/
		sshpass -p ${VNPASS} ssh nao@${VNIP} 'cd /home/nao && sudo rm -rf .temp/ && sudo src/catkin/bin/catkin_make_isolated --pkg robot_toolkit --install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/home/nao/ros_toolchain_install -j4'
		#sshpass -p ${VNPASS} ssh nao@${VNIP} 'cd /home/nao/build_isolated/robot_toolkit && sudo /home/nao/ros_toolchain_install/env.sh make -j4 && cd ../.. && sudo src/catkin/bin/catkin_make_isolated --pkg robot_toolkit --install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/home/nao/ros_toolchain_install -j4'
		echo "${GREEN}Compilation finished${NC}"
fi
if [ ${MODE} = "install" ] || [ ${MODE} = "all" ]
	then :
		echo "${YELLOW}Starting installation ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Installation failed, please check the password and IP${NC}"
				exit	
		fi
		clean_robot
		sshpass -p ${VNPASS} ssh nao@${VNIP} 'sudo mkdir -p /home/nao/.temp && sudo mkdir -p /home/nao/.temp/lib && sudo mkdir -p /home/nao/.temp/lib/pkgconfig && sudo mkdir -p /home/nao/.temp/lib/robot_toolkit &&  sudo mkdir -p /home/nao/.temp/share && sudo mkdir -p /home/nao/.temp/share/robot_toolkit && sudo  mkdir -p /home/nao/.temp/share/robot_toolkit/cmake && sudo mkdir -p /home/nao/.temp/share/robot_toolkit/launch && sudo mkdir -p /home/nao/.temp/share/qi && sudo mkdir -p /home/nao/.temp/share/qi/module && sudo cp /home/nao/ros_toolchain_install/_setup_util.py /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/env.sh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.bash /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.sh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.zsh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/.rosinstall /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/lib/pkgconfig/robot_toolkit.pc /home/nao/.temp/lib/pkgconfig && sudo cp /home/nao/ros_toolchain_install/share/robot_toolkit/cmake/robot_toolkitConfig.cmake /home/nao/.temp/share/robot_toolkit/cmake && sudo cp /home/nao/ros_toolchain_install/share/robot_toolkit/cmake/robot_toolkitConfig-version.cmake /home/nao/.temp/share/robot_toolkit/cmake && sudo cp /home/nao/ros_toolchain_install/share/robot_toolkit/package.xml /home/nao/.temp/share/robot_toolkit && sudo cp /home/nao/ros_toolchain_install/lib/librobot_toolkit_module.so /home/nao/.temp/lib && sudo cp /home/nao/ros_toolchain_install/lib/librobot_toolkit.so /home/nao/.temp/lib && sudo cp /home/nao/ros_toolchain_install/lib/robot_toolkit/robot_toolkit_node /home/nao/.temp/lib/robot_toolkit && sudo cp /home/nao/ros_toolchain_install/share/robot_toolkit/launch/robot_toolkit.launch /home/nao/.temp/share/robot_toolkit/launch && sudo cp -r /home/nao/ros_toolchain_install/share/robot_toolkit/share/ /home/nao/.temp/share/robot_toolkit/ && sudo cp /home/nao/ros_toolchain_install/share/qi/module/robot_toolkit_module.mod /home/nao/.temp/share/qi/module'
		sshpass -p ${VNPASS} scp -r nao@${VNIP}:/home/nao/.temp .temp
		sshpass -p ${ROBOTPASS} scp -r .temp/* nao@${ROBOTIP}:/home/nao/ros
		rm -r .temp
		echo "${GREEN}Binaries installed successfully${NC}"
fi
if [ ${MODE} = "run" ]
	then :
		echo "${YELLOW}Trying to run on robot ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Could not run robot_toolkit, please check the password and IP${NC}"
				exit	
		fi
		kill_process
		echo "${GREEN}Launching robot_toolkit"
		COMMAND="source /home/nao/ros/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && export ROS_HOSTNAME=$ROBOTIP && export ROS_IP=$ROBOTIP && roslaunch robot_toolkit robot_toolkit.launch"
		sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} ${COMMAND}

		exit
fi
if [ ${MODE} = "kill" ]
	then :
		echo "${YELLOW}Trying to kill robot_toolkit ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Could not kill robot_toolkit, please check the password and IP${NC}"
				exit	
		fi
		kill_process
fi
if [ ${MODE} = "clean_vn" ] || [ ${MODE} = "clean_all" ]
	then :
		clean_vn
fi
if [ ${MODE} = "clean_robot" ] || [ ${MODE} = "clean_all" ]
	then :
		clean_robot
fi

