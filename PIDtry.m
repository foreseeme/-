%利用for循环写飞行器运行仿真
global steptime
SimulationTime=100; 
steptime=0.02; %采样时间为0.02秒
T_Max=ceil(SimulationTime/steptime);%最大下标值
global pitch_max %最大俯仰角
pitch_max=pi/12;
L_lock=5;%判断是否到达目标点
V=0;w=0;%初速度、角速度
paw=0;%偏航角
pitch=0;%俯仰角
g=9.8;

Position=zeros(2,T_Max+1);
pitch_er=zeros(T_Max);
pitch_erd=zeros(T_Max);
x0=300;
y0=300;
Kpz=0.5;Kpy=0.04;Kdy=0.001;Ixy=0.019;Ixz=0.003;
C=0;

global TargetNum 
TargetNum=4; 
global Targets 
Targets=zeros(2,TargetNum);

Targets(1,1)= 300;
Targets(2,1)= 0;
Targets(1,2)= 300;
Targets(2,2)= 300;
Targets(1,3)= 0;
Targets(2,3)= 300;

CurrentNum = 1;%当前目标点
for i =1:T_Max
 %飞行器航迹控制    
 %根据目标路径给出期望姿态
    x=Position(1,i);
    y=Position(2,i);
    rxy=sqrt((x-x0)^2+(y-y0)^2);%实际位置与=距目标位置的距离
    if rxy<=L_lock
        if CurrentNum<3
            CurrentNum= CurrentNum+1;
            V=0.1;
       
        else
            break
        end    
    end
    x0=Targets(1,CurrentNum);
    y0=Targets(2,CurrentNum);

%计算期望偏航角 d下标表示期望,paw表示偏航角
    if (y0-y)>0
        paw_d=acos((x0-x)/sqrt((x0-x)^2+(y0-y)^2)); 
    else
        paw_d=2*pi-acos((x0-x)/sqrt((x0-x)^2+(y0-y)^2)); 
    end
%航向角偏差计算 er下标表示偏差
    paw_er=paw_d-paw; 
    if abs(paw_er)>=pi 
        if paw_er>0 
            paw_er=paw_er-2*pi;
        else
            paw_er=paw_er+2*pi;
        end
    end
    %PID控制 积分微分较难，暂且只用比例控制
    Mz=Kpz*paw_er;
%期望俯仰角计算 用pitch表示 C表示最小值，C小于pitch_max
    pitch_d=sqrt((x-x0)^2+(y-y0)^2)*0.1+C;
    if pitch_d > pitch_max
        pitch_d=pitch_max;
    end
    if pitch_d<-pitch_max
        pitch_d=-pitch_max;
    end
    pitch_er(i)=pitch_d-pitch;
    if i==1
        pitch_erd(i)=pitch_er(i)/steptime;
    else
        pitch_erd(i)=(pitch_er(i)-pitch_er(i))/steptime;
        
    end 
%     w_d=Kpw*pitch_er;
%     w_er=w_d-w;
    My=Kpy*pitch_er(i)+Kdy*pitch_erd(i);%比例微分控制，为方便，简化控制方式
    paw=paw+Mz*steptime^2/(2*Ixy);
    pitch=pitch+My*steptime^2/(2*Ixz);
    
    Position(1,i+1)=Position(1,i)+(g*tan(pitch)*steptime^2/2+V*steptime)*cos(paw);
    Position(2,i+1)=Position(2,i)+(g*tan(pitch)*steptime^2/2+V*steptime)*sin(paw);
    V=V+g*tan(pitch)*steptime/2;
%     w=w+My*steptime^2/(2*Ixz)/steptime;
%    Position(3,1)=   暂且不考虑高度，仅研究2维情形
end 
    plot(Position(1,:),Position(2,:))
    hold on

