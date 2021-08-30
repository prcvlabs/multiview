function filterData=ellipseDataFilter_RANSAC(data)
% Do ellipse scatter data filtering for ellipse fitting by RANSAC method.
% Author: Zhenyu Yuan
% Date: 2016/7/26
% Ref:  http://www.cnblogs.com/yingying0907/archive/2012/10/13/2722149.html
%       Extract RANSAC filtering in ellipsefit.m and make some modification
%%  ������ʼ��
nSampLen = 3;               %�趨ģ�������ݵĵ���
nDataLen = size(data, 1);   %���ݳ���
nIter = 50;                 %���ѭ������
dThreshold = 2;             %�в���ֵ
nMaxInlyerCount=-1;         %��������
A=zeros([2 1]);
B=zeros([2 1]);
P=zeros([2 1]);
%%  ��ѭ��
for i = 1:nIter 
    ind = ceil(nDataLen .* rand(1, nSampLen)); %������ѡȡnSampLen����ͬ�ĵ�
    %%  ����ģ�ͣ��洢��ģ��Ҫ�������,����͹���Բ��һ����
    %��Բ���巽�̣���������֮������Ϊ����
    A(:,1)=data(ind(1),:);    %����
    B(:,1)=data(ind(2),:);    %����
    P(:,1)=data(ind(3),:);    %��Բ��һ��
    DIST=sqrt((P(1,1)-A(1,1)).^2+(P(2,1)-A(2,1)).^2)+sqrt((P(1,1)-B(1,1)).^2+(P(2,1)-B(2,1)).^2);
    xx=[];
    nCurInlyerCount=0;        %��ʼ������Ϊ0��
    %%  �Ƿ����ģ�ͣ�
    for k=1:nDataLen
        %         CurModel=[A(1,1)   A(2,1)  B(1,1)  B(2,1)  DIST ];
        pdist=sqrt((data(k,1)-A(1,1)).^2+(data(k,2)-A(2,1)).^2)+sqrt((data(k,1)-B(1,1)).^2+(data(k,2)-B(2,1)).^2);
        CurMask =(abs(DIST-pdist)< dThreshold);     %��ֱ�߾���С����ֵ�ĵ����ģ��,���Ϊ1
        nCurInlyerCount =nCurInlyerCount+CurMask;             %���������Բģ�͵ĵ�ĸ���
        if(CurMask==1)
            xx =[xx;data(k,:)];
        end
    end
    %% ѡȡ���ģ��
    if nCurInlyerCount > nMaxInlyerCount   %����ģ�͵ĵ�������ģ�ͼ�Ϊ���ģ��
        nMaxInlyerCount = nCurInlyerCount;
        %             Ellipse_mask = CurMask;
        %              Ellipse_model = CurModel;
        %              Ellipse_points = [A B P];
        filterData =xx;
    end
end