#before using the script ensure that all programs used in the script are working
#automating play of bags and running of algorithms
#for usage on our dataset
#for fast evaluation of just one dataset with one algorithm
#roscore started for every run new
#where you want the data to be stored
destinationfolder=~/Downloads
#where keypoint.py is located and all the other python files should be located here, broad.py, and the compiled version of remap.cpp the compile guide is in the wiki
sourcefolder=/home/irobot/ownCloud/Exp/Bench
#~/ownCloud/Exp/Bench
#where launch files we additionally use lay down
additionlaunchfolder=~/ownCloud/cat/src/aros/launch
#path to bags
bagfolder=~/Downloads
#0 1 2 3 4
AR=("$bagfolder/laser2d2.bag" "$bagfolder/floor2d2.bag" "$bagfolder/odom2d2.bag" "$bagfolder/real2.bag" "$bagfolder/rectoval2d2.bag")
ARL=(40 52 67 103 78)
gtfile=("$bagfolder/laser2.txt" "$bagfolder/floor2.txt" "$bagfolder/odom2.txt" "$bagfolder/real2.txt" "$bagfolder/rectoval2.txt")
tffile=("$bagfolder/tflaser2.txt" "$bagfolder/tffloor2.txt" "$bagfolder/tfodom2.txt" "$bagfolder/tfreal2.txt" "$bagfolder/tfrectoval2.txt")
numbag=${#AR[@]}
mapfolder=~/Downloads
map=("$mapfolder/Lab_1.yaml" "$mapfolder/Lab_1.yaml" "$mapfolder/Lab_1.yaml" "$mapfolder/Lab_1.yaml" "$mapfolder/Lab_1.yaml")
#if you just want to test some startup stuff just set the time of the bag given in this array to 1
skipbag=(1 1 1 0 1)
#to get the tf data and the initialstate of odometry into a file run the script with getf=1 with your dataset once and look inside tf file given above in tffile list
#To get initalstate we take the first odometry value of the bag and take x,y,z and qx qy qz qw convert to roll pitch yaw 
#and take good care that the values below are not separated by a enter for linebreaks.
#The idenification of the initalstate of odom is necessary because for the ekf we use differential false so that the pose data is set to this value at the very start
#The reason for this ist that imu data comes to fast and would init our filter with values 0 0 0 at start and not with the correct values of the odometry. This would lead to the ekf perform bad.
#this also enables us to use just vx,vy from odometry for fusion with the x,y,qz,qw values fom vo we will not
#The functionality to set the ekf to the inital value is enable via ekf_template.yaml file which is used by ekf_template2.launch

#we can keep the camera data to zeros and avoid by this having to apply this tf on the gt data to get gt in base_footprint
camtoopt=('"1579261351.192527056, 0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5"')
basetocam=('"1579261351.192527056, 0.325, 0.0175, 0.225, 0, 0, -0.707, 0.707"')
basetolidar=('"1579261351.192527056, 0.35, -0.26, 0.06, 0, 0, -0.39497015, 0.9186395"')

#amount of repetitions
maxrep=1
#delay of bag to enable subscribers to register to topics 
#delay should be at least 3 seconds otherwise for example robot_localization will not have enough time to get the starting position right
delay=3
#rate at which the bag shall be played backe
rate=(1 1 1 1 0.1 0.1 0.1 1 1 1 1 1 1 1 1 0.1)
#for rtabmap run the removeduplicate.py script to ensure no duplicates get published.
algnam=('wheelodom' 'lidarhector' 'amcl' 'lidargmap' 'rtabmap' 'ekfall' 'orb' 'imu' 'lidarohm' 'monovo' 'mrpt-icpslam' 'lsdslam' 'paslam' 'mrpt' 'viso' 'karto')
#if the order above is changed changes need to be made to keypoint.py and keypoint.py as well for odom i set mal to 0 but also passed 3 to keypoint cause gmapping was running it.
mal=-1
#which algorithm to skip
skip=(1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1)
#if you want to get the transforms maptoodom maptopose set this to true then change the above variables tracktobase odomtobasefoot camtoopt basetocam must be know
getf=0
if (( $getf == 1))
then
for ((idx=0; idx<$numbag; idx++))
do    
if (( ${skipbag[idx]} == 0))
then
	xterm -hold -e "python $sourcefolder/keypoint.py 1 ${AR[idx]} ${gtfile[idx]} ${tffile[idx]}"
	sleep 1
fi
done
exit
fi
#internal parameters
a=""
pause=0
#reduce it with 1 because it starts k with 0 
numofalgo=$(echo  "(${#algnam[@]}-1)"|bc )
#https://unix.stackexchange.com/questions/278502/accessing-array-index-variable-from-bash-shell-script-loop
for ((idx=0; idx<$numbag; idx++))
do
    if (( ${skipbag[idx]} == 0))
    then
	#get the tfs from the tf files above mentioned under tffile, which are provided in the last line of this part of the programm
	k=0
	while read -r line; do
	#https://stackoverflow.com/questions/9084257/bash-array-with-spaces-in-elements
	test=($f)
	testr=''
	for m in ${line[*]}
	do
		testr=($echo"$testr $m")
	done
	if (($k == 0))
	then 
		initalstate=("${testr[*]}")
	fi
	if (($k == 1))
	then 
		maptopose=("${testr[*]}")
	fi
	if (($k == 2))
	then 
		odomtobasefoot=("${testr[*]}")
	fi
	if (($k == 3))
	then 
		maptoodom=("${testr[*]}")
	fi
        k=$(echo "($k + 1)"|bc ) 
	done < ${tffile[idx]}

    #https://stackoverflow.com/questions/9778741/writing-a-bash-for-loop-with-a-variable-top-end
    xterm -e "rosbag info ${AR[idx]}" &
    rosbagload=$!           
    sleep 1
    while ps -p $rosbagload > /dev/null
    do
        echo "Loading bag"
    done              
    for ((k=0; k<=$numofalgo; k++))
    do       
    if (( ${skip[k]} == 0))
    then
    for ((rep=0; rep<maxrep; rep++))
    do   
                xterm -e "rosclean purge -y"  
                xterm -title "roscore" -e "roscore" &
                ID=$!
                sleep 7   
		#everytime orb is involved
                if (($k == 6)) ||  (($k == 5)) ||  (($k == 15))
                then   		
    			xterm -hold -e "rostopic echo /camera/depth_registered/image_rect_raw/header" &
                	myBackgroundXtermPID6=$!
		fi 
                xterm -hold -e "rosparam set use_sim_time true;python $sourcefolder/broad.py 0" &
                myBackgroundXtermPID1=$!
                amcladdsid=$myBackgroundXtermPID1
                ekf1id=$myBackgroundXtermPID1
                ekf2id=$myBackgroundXtermPID1
                ekf3id=$myBackgroundXtermPID1
                ekf4id=$myBackgroundXtermPID1
                ekf5id=$myBackgroundXtermPID1
                #sleep above is necessary so that broad.py comes up
                sleep 1                
                #for wheelodom
                if (($k == 0))
                then                  
                    #automatically closes no need for kill
                    xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam} 0 ${mal}" &
                    myBackgroundXtermPID6=$!
                fi
                #for hectorslam
                if (($k == 1))
                then     
                    a="roslaunch $additionlaunchfolder/hectormapping_default.launch"
                fi                
                #for amcl
                if (($k == 2))
                then
                    #check if the maplaunch worked otherwise amcl will not get a map
                    xterm -e "roslaunch $additionlaunchfolder/mapserver.launch map_file:=${map[idx]}"&
                    amcladdsid=$!   
                    a="roslaunch $additionlaunchfolder/amcl.launch"                    
                fi 
                #for gmapping
                if (($k == 3))
                then
                    a="roslaunch $additionlaunchfolder/gmap.launch"  
                fi     
		#for rtabmap
                if (($k == 4))
                then
		    #camera_floor is the name of the topic published by the floor camera                                   
		    xterm -hold -e "python $sourcefolder/rtabmapchange.py" &
                    ekf5id=$!   
		    a="roslaunch rtabmap_ros rtabmap.launch"
                fi 
		#ekf
                if (($k == 5))
                then   		
	            xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam} 0 ${mal}" &
                    ekf1id=$!      
	            xterm -hold -e "roslaunch realsense2_camera opensource_tracking2.launch" &
                    ekf3id=$!   
		    #for the floorfacing camera
	            xterm -hold -e "roslaunch rtabmap_ros rtabmap.launch" &
                    ekf4id=$!              
   		    xterm -e "roslaunch $additionlaunchfolder/gmap.launch" &
                    amcladdsid=$!                      
                    scaleFactor=1.25
                    nLevels=8
                    iniThFast=20
                    minThFast=6
                    #this script assume that orbslam src folder is /home/irobot/catkin_ws/src/orb_slam_2_ros/ adopt it if necessary
                    xterm -e  "python $sourcefolder/eyamlorb.py $scaleFactor $nLevels $iniThFast $minThFast"
                    sleep 2
                    a="roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch"
                    #When applying changes to the yaml file keep in mind that some vo algorithms publish to the topic vo2 while others to the topic vop because they have different message types
                    #so when editing stuff for the config also of the one topic apply the same changes to the conf to the topic vop
                    xterm -hold -e "roslaunch robot_localization ekf_template2.launch inital:='$initalstate'" &
                    ekfPID=$!   
                fi 
                #for orbslam
                if (($k == 6))
                then
		    #seems to be essential otherwise orbslam seems to stop publishing 
                    scaleFactor=1.25
                    nLevels=8
                    iniThFast=6
                    minThFast=3
                    #this script assume that orbslam src folder is /home/irobot/catkin_ws/src/orb_slam_2_ros/ adopt it if necessary
                    xterm -e  "python $sourcefolder/eyamlorb.py $scaleFactor $nLevels $iniThFast $minThFast"
                    sleep 2
                    a="roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch"
                fi
		#for imu
                if (($k == 7))
                then
                    xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam} 0 ${mal}" &
                    myBackgroundXtermPID6=$!  
                    #automatically closes no need for kill
		    # by providing the necessary information for transformation of the imu data into the odometry frame the ekf will take 
		    # care to the imu data being in the right coordinate system. so there is no need to transform the data on your own
		    # only the transformation from camera_link into camera_imu_ and from basefootprint to camera_link are necessary
		    # we provide them within broad.py
                    # the reason for this is that imu_filter_magic publish its data relative to the parent frame which is by default odom
                    a="roslaunch realsense2_camera opensource_tracking2.launch" 
                fi 
 		#for ohm_tsd_slam #needs tf odom to base 
                if (($k == 8))
                then
                    a="roslaunch ohm_tsd_slam slam2.launch"
                fi 
		#for monovo
                if (($k == 9))
                then
                    a="python $sourcefolder/monovo.py" 
                fi
                #for mrpt icp slam
                if (($k == 10))
                then
                    a="roslaunch mrpt_icp_slam_2d icp_slam.launch"
                fi 
                #for lsd_slam_core
                if (($k == 11))
                then
                    a="rosrun lsd_slam_core live_slam /image:=/camera_floor/int/image_raw /camera_info:=/camera_floor/int/camera_info"
                fi 
		#for paslam
                if (($k == 12))
                then                          
                    xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam}" &
                    myBackgroundXtermPID6=$!
                    a="roslaunch pa_slam pa_slam.launch" 
                fi 
		#for mrpt
                if (($k == 13))
                then
                    a="roslaunch mrpt_localization demoevo.launch map_file:=map.gridmap"
                fi 
		#for viso2
                if (($k == 14))
                then
                    a="roslaunch viso2_ros demo.launch"
                fi
		#for karto
                if (($k == 15))
                then                          
                    xterm -hold -e "python $sourcefolder/wheel.py ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetocam}" &
                    myBackgroundXtermPID6=$!
                    a="roslaunch $additionlaunchfolder/karto.launch" 
                fi 
                xterm -hold -e "python $sourcefolder/keypoint.py 0 ${maptoodom[0]} ${maptopose[0]} ${odomtobasefoot[0]} ${camtoopt} ${basetolidar} $k $mal ${basetocam}" &
                myBackgroundXtermPID0=$!
                xterm -hold -e $a &
                myBackgroundXtermPID4=$!
                xterm -hold -e "$sourcefolder/./remap" &
                remap=$!  		
		#ensure that all scripts are up before playing the bag
                xterm -hold -e "rosparam set use_sim_time true;rosbag play -k --clock -d ${delay} -r ${rate[k]} ${AR[idx]} /odom:=/odomc /tf:=/tf2" &
                rosbagPID5=$! 
		sleep 12
		#https://unix.stackexchange.com/questions/89712/how-to-convert-floating-point-number-to-integer  
		#the bonus of one seconds ensures that rounding problems from calculation of the time of the bagplay will not have an effect
                sleptim=$(echo "(${ARL[idx]}+1)/${rate[k]}"|bc ) 
                sleptim=$(echo "${sleptim%.*}+${delay}"|bc)  
                for ((time=0; time<=$((sleptim)); time=time+1))
                do
                    echo "Evaluating ${algnam[k]} current time elapsed ${time} to go ${sleptim}  on bag number ${idx} Totaltime of bag is extend with 1 seconds devided by playrate to take care of rounding effects"
                    echo "If you want to pause press p"
                    #https://stackoverflow.com/questions/5542016/bash-user-input-if #https://askubuntu.com/questions/446156/pause-execution-and-wait-for-user-input
                    #https://stackoverflow.com/questions/9483633/how-to-use-bash-read-with-a-timeout
                    mainmenuinput='k'
                    read -n 1 -t 1 mainmenuinput
                    if [[ $mainmenuinput == 'p' ]];
                    then
                        pause=1
                        echo $pause
                    fi
                    if (($k==8)) && (($(echo "(1+$time)"| bc -l) == $((sleptim))))
	            then
		    	xterm -e "rosrun map_server map_saver -f ohmtsd"
                    fi       
                done
                if ((k==0))
                then
                    #store gt and odom
                    mv $sourcefolder/wheelodom.txt "$destinationfolder/$idx-odom-$rep.txt"
                fi              
                if ((k==1)) #hector
                then
		    xterm -e "rosrun map_server map_saver -f hectormap"
                    mv $sourcefolder/hectormap.pgm "$destinationfolder/$idx-hectormap-$rep.pgm"
                    mv $sourcefolder/hectormap.yaml "$destinationfolder/$idx-hectormap-$rep.yaml"
                    mv $sourcefolder/hector.txt "$destinationfolder/$idx-hector-$rep.txt"
                fi                
                if ((k==2)) #amcl
                then
                    kill $amcladdsid
                    kill $ekf1id
                    mv $sourcefolder/amcl.txt "$destinationfolder/$idx-amcl-$rep.txt"
                fi
                if ((k==3)) #gmap
                then
		    xterm -e "rosrun map_server map_saver -f gmap"
                    mv $sourcefolder/gmap.txt "$destinationfolder/$idx-gmap-$rep.txt"
		    mv $sourcefolder/gmap.pgm "$destinationfolder/$idx-gmap-$rep.pgm"
		    mv $sourcefolder/gmap.yaml "$destinationfolder/$idx-gmap-$rep.yaml"
                fi  
                if ((k==4)) #rtabmap
                then                 
                    mv $sourcefolder/rtabmap.txt "$destinationfolder/$idx-rtabmap-$rep.txt"
                    kill $ekf5id
                fi               
                if ((k==5)) #ekf all
                then                 
                    mv $sourcefolder/ekf_pose.txt "$destinationfolder/$idx-ekf_pose-$rep.txt"
                    kill $ekf1id
                    kill $ekf2id
                    kill $ekf3id
                    kill $ekf4id
                    kill $ekf5id
                    kill $amcladdsid
                fi  
                if ((k==6)) #orb
                then                 
                    mv $sourcefolder/visual_odom_orb.txt "$destinationfolder/$idx-orbslam2-$rep.txt"
                fi  
                if ((k==7))
                then
                    mv $sourcefolder/imupose.txt "$destinationfolder/$idx-imupose-$rep.txt"
                fi
 		if ((k==15)) #karto
                then
               	    xterm -e "rosrun map_server map_saver -f karto"
                    mv $sourcefolder/karto.txt "$destinationfolder/$idx-karto-$rep.txt"
                    mv $sourcefolder/karto.pgm "$destinationfolder/$idx-karto-$rep.pgm"
		    mv $sourcefolder/karto.yaml "$destinationfolder/$idx-karto-$rep.yaml"
                fi
                if ((k==9)) #mrpt
                then                 
                    mv $sourcefolder/monovo.txt "$destinationfolder/$idx-monovo-$rep.txt"
                fi  
                if ((k==10)) #mrpt-icpslam
                then                 
                    mv $sourcefolder/mrpticpslam.txt "$destinationfolder/$idx-mrpticpslam-$rep.txt"
                    xterm -e "rosrun map_server map_saver -f mrpicpslam"
                    mv $sourcefolder/mrpicpslam.pgm "$destinationfolder/$idx-mrpicpslam-$rep.pgm"
		    mv $sourcefolder/mrpicpslam.yaml "$destinationfolder/$idx-mrpicpslam-$rep.yaml"
                fi  
                if ((k==11)) #lsdslam
                then                 
                    mv $sourcefolder/lsdslam.txt "$destinationfolder/$idx-lsdslam-$rep.txt"
                fi  
		if ((k==12)) #paslam
                then                 
               	    xterm -e "rosrun map_server map_saver map:=/paslam/probmap -f pa"
                    mv $sourcefolder/pa_slam.txt "$destinationfolder/$idx-pa_slam-$rep.txt"
                    mv $sourcefolder/pa.pgm "$destinationfolder/$idx-pa-$rep.pgm"
		    mv $sourcefolder/pa.yaml "$destinationfolder/$idx-pa-$rep.yaml"
                fi  
		if ((k==13)) #mrpt
                then                 
                    mv $sourcefolder/mrpt.txt "$destinationfolder/$idx-mrpt-$rep.txt"
                fi 
		if ((k==14)) #viso2
                then                 
                    mv $sourcefolder/viso2.txt "$destinationfolder/$idx-viso2-$rep.txt"
                fi   		   
 		if ((k==8)) #ohm
                then
		    #mapserver will be started above because the bag needs to be running / data needs to come in for ohm to publish the map
		    mv $sourcefolder/ohmtsd.pgm "$destinationfolder/$idx-ohmtsd-$rep.pgm"
		    mv $sourcefolder/ohmtsd.yaml "$destinationfolder/$idx-ohmtsd-$rep.yaml"
                    mv $sourcefolder/ohmtsd.txt "$destinationfolder/$idx-ohmtsd-$rep.txt"
                fi
                kill $ID
                kill $myBackgroundXtermPID0
                kill $myBackgroundXtermPID1
                kill $remap
                kill $ekfPID
                kill $myBackgroundXtermPID4
                kill $rosbagPID5 
                kill $myBackgroundXtermPID6 
                if (($pause == 1))
                then
                    kill $ID
                    mainmenuinput='k'                    
                    while [[ $mainmenuinput != 'u' ]]
                    do
                        echo "want to continue? press u"
                        read -n 1 -t 20 mainmenuinput  
                    done
                    pause=0   
                fi
    done 
    fi
    done
fi
done

kill $ID
