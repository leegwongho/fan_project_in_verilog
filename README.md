목표

![image](https://github.com/user-attachments/assets/44db7078-4699-4223-b103-99266294100b)

팀 R&R

![image](https://github.com/user-attachments/assets/7639ffd5-539b-471e-96f1-e2f8b1055fc9)

프로젝트 계획

![image](https://github.com/user-attachments/assets/b407c009-0bb4-452e-b2f7-c6928c1bb7a6)

1차 목표

![image](https://github.com/user-attachments/assets/08f37f8c-c70a-4470-8de7-de38be49baed)

추가할 기능

![image](https://github.com/user-attachments/assets/2f3b42db-1de7-49b8-a666-6f05ee22dbd4)


![image](https://github.com/user-attachments/assets/4d37ae2a-5377-4e8d-abbc-a8b74207ca14)

가변 저항으로 타이머 입력을 받을수 있도록 설계했다. VIVADO 내에서 256번 평균을 내는 대에도 가변 저항의 값이 많이 흔들려서 16번 평균을 더내고

하위 3비트를 잘라서 값을 사용해야 했다. 

![image](https://github.com/user-attachments/assets/b13e751f-19db-4ab4-8af9-d989073ead42)

타이머 이다. fsm을 이용하여 1시간 3시간 5시간 알람을 설정 가능하게 하였고 테스트를위해 초단위로 변경해둔 상태이다.  0이되면 모터 스탑비트를 출력으로

내보내도록 설계하였다. 또 롱키 입력을 받아 가변저항으로도 시간을 설정가능하게 하였다. 

![image](https://github.com/user-attachments/assets/3a7afe06-a2e8-4e13-9e83-ba92a02c1528)

안전을위한 초음파 센서이다. 물체나 인체가 가까워지면 모터가 정지한다.

![image](https://github.com/user-attachments/assets/d408e9c7-0284-4f83-a063-239f972a8b7d)

사람 디텍터 이다. 

servo 모터의 왼쪽 가운데 오른쪽 에 초음파 센서를 장착한다.

왼쪽이나 오른쪽에 물체가 감지되면 각 방향으로 servo 모터를 회전시키고 가운데에 물체가 감지되면 정지하도록 설계하였다.

처음엔 servo모터가 너무빨라 초음파가 물체를 감지하기도전에 각방향의 끝으로 동작하여 문제가 생겼고 해결 이후 각초음파에 동시에 물체가 감지될경우를

우선 순위를 생각하지 않고 설계하여 문제가 생겨 각 값을 비교하여 가장 가까운 값으로 동작하도록 하여 해결했다.

![image](https://github.com/user-attachments/assets/c320e2f9-2f83-43c0-ba07-700202245f0a)


top 모듈의 회로도이다. 

팀장을 맡은것이 처음이다보니 많이 미숙했다. 팀원에게 모듈을 만들때에있어 탑모듈에서 합칠때를 상정하고 만들어 달라 했어야 했는데 그러지않아 서로 각자의 동작만 해 문제가 생겼다.


시연 영상 https://youtube.com/playlist?list=PLo8EdZz0OIEMR5owuCp1N2yK9hUTOdl70&feature=shared

[harman_fan_Project_verilog.pptx](https://github.com/user-attachments/files/16377806/harman_fan_Project_verilog.pptx)
