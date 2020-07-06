%����forѭ��д���������з���
global steptime
SimulationTime=100; 
steptime=0.02; %����ʱ��Ϊ0.02��
T_Max=ceil(SimulationTime/steptime);%����±�ֵ
global pitch_max %�������
pitch_max=pi/12;
L_lock=5;%�ж��Ƿ񵽴�Ŀ���
V=0;w=0;%���ٶȡ����ٶ�
paw=0;%ƫ����
pitch=0;%������
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

CurrentNum = 1;%��ǰĿ���
for i =1:T_Max
 %��������������    
 %����Ŀ��·������������̬
    x=Position(1,i);
    y=Position(2,i);
    rxy=sqrt((x-x0)^2+(y-y0)^2);%ʵ��λ����=��Ŀ��λ�õľ���
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

%��������ƫ���� d�±��ʾ����,paw��ʾƫ����
    if (y0-y)>0
        paw_d=acos((x0-x)/sqrt((x0-x)^2+(y0-y)^2)); 
    else
        paw_d=2*pi-acos((x0-x)/sqrt((x0-x)^2+(y0-y)^2)); 
    end
%�����ƫ����� er�±��ʾƫ��
    paw_er=paw_d-paw; 
    if abs(paw_er)>=pi 
        if paw_er>0 
            paw_er=paw_er-2*pi;
        else
            paw_er=paw_er+2*pi;
        end
    end
    %PID���� ����΢�ֽ��ѣ�����ֻ�ñ�������
    Mz=Kpz*paw_er;
%���������Ǽ��� ��pitch��ʾ C��ʾ��Сֵ��CС��pitch_max
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
    My=Kpy*pitch_er(i)+Kdy*pitch_erd(i);%����΢�ֿ��ƣ�Ϊ���㣬�򻯿��Ʒ�ʽ
    paw=paw+Mz*steptime^2/(2*Ixy);
    pitch=pitch+My*steptime^2/(2*Ixz);
    
    Position(1,i+1)=Position(1,i)+(g*tan(pitch)*steptime^2/2+V*steptime)*cos(paw);
    Position(2,i+1)=Position(2,i)+(g*tan(pitch)*steptime^2/2+V*steptime)*sin(paw);
    V=V+g*tan(pitch)*steptime/2;
%     w=w+My*steptime^2/(2*Ixz)/steptime;
%    Position(3,1)=   ���Ҳ����Ǹ߶ȣ����о�2ά����
end 
    plot(Position(1,:),Position(2,:))
    hold on

