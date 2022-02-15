roslaunch px4 trial_2.launch &
sleep 37
python ~/XTDrone/communication/multirotor_communication.py iris 0 &
sleep 1 
python red_ball_tracking_trial_2.py &
sleep 5
rosrun rqt_gui rqt_gui --perspective-file image_with_circle.perspective &
python red_ball_detection.py 