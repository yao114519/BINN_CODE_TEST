# 生物启发神经网络BINN 全覆盖路径规划 + 改进A*算法解决死锁点 + 规则移动法解决转向问题
### 未修改A*算法产生路径
<img width="665" height="694" alt="eac433f036b2a69de527d12d59acdb57" src="https://github.com/user-attachments/assets/edd1fc25-075d-4f27-ada9-5a713851cd16" />
在（7，16）和（8，17）点处出现拐点，增加路径复杂度
### 修改A*算法产生路径
<img width="684" height="663" alt="image" src="https://github.com/user-attachments/assets/9f3d1c38-e93b-446f-897d-0cd021578678" />
增加障碍物，增加死锁点，增加斜向移动规则，始终是保持沿y轴前进
