# basler-ros-driver

### - 파라미터 설정(config/default.yaml)

​	`camera_count` : 카메라 개수

​	`camera1` : 첫번째 카메라 설정 (개수가 늘어나면 camera2, camera3 ...)

​		`name `  : 해당되는 카메라의 이미지가 저장될 때의 폴더 이름

​		`serial_number`  : 해당되는 카메라의 시리얼 번호 



### - 노드 실행

​	`$ roslaunch basler_ros_driver run.launch`



### - Publish 메세지

​	`basler_ros_driver/camera_info` : 현재 실행되는 카메라에 대한 정보

​	`basler_ros_driver/camera1/image_raw` : 1번 카메라의 촬영된 이미지 데이터 (번호는 yaml파일 설정에 따라)



### - 서비스 List	

|          Service Name          |                 Notes                  |
| :----------------------------: | :------------------------------------: |
|   basler_ros_driver/grabbing   | value : 0 = stop, 1 = start **(bool)** |
|   basler_ros_driver/trigger    |  value : 1 = send trigger **(bool)**   |
| basler_ros_driver/set_save_dir | image save directory path **(string)** |



​	