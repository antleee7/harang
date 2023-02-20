# harang
2023 HARANG Avionics

하랑에 탑재될 코드 및 지상국 코드입니다.

크게 3개의 루프로 구성돼있는데, 첫째 루프는 센서의 raw data를 sd카드에 기록하는 루프고, 둘째 루프는 raw data를 바탕으로 칼만 필터링을 하고, processed data를 sd카드에 기록하는 루프입니다. 마지막 루프는 메인 루프로, data uplink 및 칼만 data downlink, 외에 모든 로켓의 진행 단계를 담당합니다.