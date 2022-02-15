roslaunch px4 trial_1.launch &
sleep 26
python ~/XTDrone/communication/multirotor_communication.py iris 0 &
sleep 1 
python red_ball_tracking_trial_1.py &
sleep 5
rosrun rqt_gui rqt_gui --perspective-file image_with_circle.perspective &
python red_ball_detection.py 