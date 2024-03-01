%
clear rob data rob1 rob2 rob3 rob4 rob5 rob6 rob7 rob8 rob9 rob10
clear rob_ind1 rob_ind2 rob_ind3 rob_ind4 rob_ind5 rob_ind6 rob_ind7 rob_ind8 rob_ind9 rob_ind10
clear rob_joint1 rob_joint2 rob_joint3 rob_joint4 rob_joint5 rob_joint6 rob_joint7 rob_joint8 rob_joint9 rob_joint10
clear xk w1 w2 w3 w4 w5 w6 w7 w8 w9 w10
rng(3);
num_samples=1e3;
dist = makedist('normal','mu',0,'sigma',0.1);
xk{1}={[x01 zeros(2,100)],[x02 zeros(2,100)],[x03 zeros(2,100)],[x04 zeros(2,100)],[x05 zeros(2,100)],[x06 zeros(2,100)],...
    [x07 zeros(2,100)],[x08 zeros(2,100)],[x09 zeros(2,100)],[x010 zeros(2,100)]};
for i=1:num_samples
    xk{i}={[x01 zeros(2,100)],[x02 zeros(2,100)],[x03 zeros(2,100)],[x04 zeros(2,100)],[x05 zeros(2,100)],[x06 zeros(2,100)],...
    [x07 zeros(2,100)],[x08 zeros(2,100)],[x09 zeros(2,100)],[x010 zeros(2,100)]};
    w1=random(dist,2,101); w2=random(dist,2,101); w3=random(dist,2,101); w4=random(dist,2,101); w5=random(dist,2,101);
    w6=random(dist,2,101); w7=random(dist,2,101); w8=random(dist,2,101); w9=random(dist,2,101); w10=random(dist,2,101);
    for j=1:100
        xk{i}{1}(:,j+1)=A*xk{i}{1}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{1}(:,j)-x1(:,j)))+w1(:,j);
        xk{i}{2}(:,j+1)=A*xk{i}{2}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{2}(:,j)-x2(:,j)))+w2(:,j);
        xk{i}{3}(:,j+1)=A*xk{i}{3}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{3}(:,j)-x3(:,j)))+w3(:,j);
        xk{i}{4}(:,j+1)=A*xk{i}{4}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{4}(:,j)-x4(:,j)))+w4(:,j);
        xk{i}{5}(:,j+1)=A*xk{i}{5}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{5}(:,j)-x5(:,j)))+w5(:,j);
        xk{i}{6}(:,j+1)=A*xk{i}{6}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{6}(:,j)-x6(:,j)))+w6(:,j);
        
        xk{i}{7}(:,j+1)=A*xk{i}{7}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{7}(:,j)-x7(:,j)))+w7(:,j);
        xk{i}{8}(:,j+1)=A*xk{i}{8}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{8}(:,j)-x8(:,j)))+w8(:,j);
        xk{i}{9}(:,j+1)=A*xk{i}{9}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{9}(:,j)-x9(:,j)))+w9(:,j);
        xk{i}{10}(:,j+1)=A*xk{i}{10}(:,j)+B*(v1(:,j)+Kgain*(xk{i}{10}(:,j)-x10(:,j)))+w10(:,j);
    end
end
%

figure(3); hold on;
for i=1:num_samples
    plot(xk{i}{1}(1,:),xk{i}{1}(2,:),'-','linewidth',1., 'Color',[0 0.5 0.5])
    plot(xk{i}{2}(1,:),xk{i}{2}(2,:),'-','linewidth',1., 'Color',[0 1 0])
    plot(xk{i}{3}(1,:),xk{i}{3}(2,:),'-','linewidth',1., 'Color',[0 0 1])
    plot(xk{i}{4}(1,:),xk{i}{4}(2,:),'-','linewidth',1., 'Color',[0 1 1])
    plot(xk{i}{5}(1,:),xk{i}{5}(2,:),'-','linewidth',1., 'Color',[1 0 1])
    plot(xk{i}{6}(1,:),xk{i}{6}(2,:),'-','linewidth',1., 'Color',[1 1 0])
    plot(xk{i}{7}(1,:),xk{i}{7}(2,:),'-','linewidth',1., 'Color',[1 0.5 0])
    plot(xk{i}{8}(1,:),xk{i}{8}(2,:),'-','linewidth',1., 'Color',[0.5 0 0.5])
    plot(xk{i}{9}(1,:),xk{i}{9}(2,:),'-','linewidth',1., 'Color',[1 0.8 0.8])
    plot(xk{i}{10}(1,:),xk{i}{10}(2,:),'-','linewidth',1., 'Color',[0.64 0.16 0.16])
end

for i=1:num_samples
    [rob1{i}, rob_ind1{i}, rob_joint1{i}] = compute_robustness_three_edges(xk{i}{1},xk{i}{2},xk{i}{3},xk{i}{5},C1,c1,prox,prox,prox,t1,g1,j1,N);
    [rob2{i}, rob_ind2{i}, rob_joint2{i}] = compute_robustness_two_edges(xk{i}{2},xk{i}{1},xk{i}{3},C2,c2,prox,prox,t2,g2,j2,N);
    [rob3{i}, rob_ind3{i}, rob_joint3{i}] = compute_robustness_three_edges(xk{i}{3},xk{i}{1},xk{i}{2},xk{i}{4},C3,c3,prox,prox,prox,t3,g3,j3,N);
    [rob4{i}, rob_ind4{i}, rob_joint4{i}] = compute_robustness_three_edges(xk{i}{4},xk{i}{3},xk{i}{5},xk{i}{7},C4,c4,prox,prox,prox,t4,g4,j4,N);
    [rob5{i}, rob_ind5{i}, rob_joint5{i}] = compute_robustness_three_edges(xk{i}{5},xk{i}{1},xk{i}{4},xk{i}{6},C5,c5,prox,prox,prox,t5,g5,j5,N);
    [rob6{i}, rob_ind6{i}, rob_joint6{i}] = compute_robustness_three_edges(xk{i}{6},xk{i}{5},xk{i}{8},xk{i}{9},C6,c6,prox,prox,prox,t6,g6,j6,N);
    [rob7{i}, rob_ind7{i}, rob_joint7{i}] = compute_robustness_two_edges(xk{i}{7},xk{i}{4},xk{i}{8},C7,c7,prox,prox,t7,g7,j7,N);
    [rob8{i}, rob_ind8{i}, rob_joint8{i}] = compute_robustness_three_edges(xk{i}{8},xk{i}{6},xk{i}{7},xk{i}{10},C8,c8,prox,prox,prox,t8,g8,j8,N);
    [rob9{i}, rob_ind9{i}, rob_joint9{i}] = compute_robustness_two_edges(xk{i}{9},xk{i}{6},xk{i}{10},C9,c9,prox,prox,t9,g9,j9,N);
    [rob10{i}, rob_ind10{i}, rob_joint10{i}] = compute_robustness_two_edges(xk{i}{10},xk{i}{8},xk{i}{9},C10,c10,prox,prox,t10,g10,j10,N);
%     rob(i)=min([rob1{i} rob2{i} rob3{i} rob4{i} rob5{i} rob6{i} rob7{i} rob8{i} rob9{i} rob10{i}]);
%     rob(i)=min([rob_ind1{i} rob_ind2{i} rob_ind3{i} rob_ind4{i} rob_ind5{i} rob_ind6{i}]);
end
%
for i=1:num_samples
    rob(i)=min([min(rob_ind1{i},rob_joint1{i}), min(rob_ind2{i},rob_joint2{i}),min(rob_ind3{i},rob_joint3{i}),...
        min(rob_ind4{i},rob_joint4{i}),min(rob_ind5{i},rob_joint5{i}),min(rob_ind6{i},rob_joint6{i}),...
        min(rob_ind7{i},rob_joint7{i}),min(rob_ind8{i},rob_joint8{i}),min(rob_ind9{i},rob_joint9{i}),min(rob_ind10{i},rob_joint10{i})]);
end
close all
fontsize=16;
% figure(1);
% O1.plot('color','red'); hold on;
% O2.plot('color', 'red');
% O3.plot('color', 'red');
% T1.plot('color',[0 0.5 0.5]); G1.plot('color',[0 0.5 0.5]);     % teal color
% T2.plot('color',[0 1 0]); G2.plot('color',[0 1 0]);             % green color
% T3.plot('color',[0 0 1]); G3.plot('color',[0 0 1]);             % blue color
% T4.plot('color',[0 1 1]); G4.plot('color',[0 1 1]);             % cyan color
% T5.plot('color', [1 0 1]); G5.plot('color', [1 0 1]);           % magenta color
% T6.plot('color',[1 1 0]); G6.plot('color',[1 1 0]);             % yellow color
% T7.plot('color',[1 0.5 0]); G7.plot('color',[1 0.5 0]);         % orange color
% T8.plot('color',[0.5 0 0.5]); G8.plot('color',[0.5 0 0.5]);     % purple color
% T9.plot('color',[1 0.8 0.8]); G9.plot('color',[1 0.8 0.8]);     % pink color
% T10.plot('color',[0.64 0.16 0.16]); G10.plot('color',[0.64 0.16 0.16]); % brown color
% axis([-1 50 -1 22.5]);
% ax = gca;
% title('Workspace','FontSize',fontsize,'Interpreter','Latex')
% %
% figure(1); hold on;
% for i=1:num_samples
% plot(xk{i}{1}(1,:),xk{i}{1}(2,:),'*-','linewidth',1.5, 'Color',[0 0.5 0.5])
% plot(xk{i}{2}(1,:),xk{i}{2}(2,:),'*-','linewidth',1.5, 'Color',[0 1 0])
% plot(xk{i}{3}(1,:),xk{i}{3}(2,:),'*-','linewidth',1.5, 'Color',[0 0 1])
% plot(xk{i}{4}(1,:),xk{i}{4}(2,:),'*-','linewidth',1.5, 'Color',[0 1 1])
% plot(xk{i}{5}(1,:),xk{i}{5}(2,:),'*-','linewidth',1.5, 'Color',[1 0 1])
% plot(xk{i}{6}(1,:),xk{i}{6}(2,:),'*-','linewidth',1.5, 'Color',[1 1 0])
% plot(xk{i}{7}(1,:),xk{i}{7}(2,:),'*-','linewidth',1.5, 'Color',[1 0.5 0])
% plot(xk{i}{8}(1,:),xk{i}{8}(2,:),'*-','linewidth',1.5, 'Color',[0.5 0 0.5])
% plot(xk{i}{9}(1,:),xk{i}{9}(2,:),'*-','linewidth',1.5, 'Color',[1 0.8 0.8])
% plot(xk{i}{10}(1,:),xk{i}{10}(2,:),'*-','linewidth',1.5, 'Color',[0.64 0.16 0.16])
% end
% hold off;
% data=[];
% for i=1:num_samples
%     if rob(i)>-0.18
%         data(i)=2e-9;
%     else
%         data(i)=rob(i)+0.12;
%     end
% end
% Sample data
%rob=rob+3.5;
figure(2)
data = rob;
% for i =1:length(data)
%     coin=rand;
%     if coin<0.05
%         data(i)=data(i);
%     else
%         data(i)=1e-2*rand;
%     end
% end
clear data
data=rob;

% for i=1:num_samples
%     if  data(i)<0
%         data(i)=0;
%     end
% end
% Specify the bin edges
edges = min(data):.0001:max(data);  % Adjust the bin edges based on your data range

% Compute the histogram counts
[counts, binEdges] = histcounts(data, edges,'Normalization','probability');

% Create a bar plot to visualize the histogram
bar(binEdges(1:end-1), counts, 'BarWidth', 0.8, 'FaceColor', 'blue', 'EdgeColor', 'black');

% Add labels and title
xlabel('Multi-agent robustness function $\rho^\phi$','FontSize',fontsize,'Interpreter','Latex');
ylabel('Frequency','FontSize',fontsize,'Interpreter','Latex');
%title('Histogram with Frequency of Observations');

% Display grid
grid on;


% robustness for agents with two edges
function [rob, rob_ind, rob_joint] = compute_robustness_two_edges(x,xk1,xk2,C,c,e1,e2,t,g,j,N)
%
robO1=[]; robO2=[]; robO3=[]; robO=[];
robT=[];
robG=[];
rob_1=[];
rob_2=[];
for i=1:N+1
    robO1(i)=min([C(1,:)*x(:,i)+c(1) C(2,:)*x(:,i)+c(2) C(3,:)*x(:,i)+c(3) C(4,:)*x(:,i)+c(4)]);
    robO2(i)=min([C(5,:)*x(:,i)+c(5) C(6,:)*x(:,i)+c(6) C(7,:)*x(:,i)+c(7) C(8,:)*x(:,i)+c(8)]);
    robO3(i)=min([C(9,:)*x(:,i)+c(9) C(10,:)*x(:,i)+c(10) C(11,:)*x(:,i)+c(11) C(12,:)*x(:,i)+c(12)]);
end
rob11=-max(robO1); rob12=-max(robO2); rob13=-max(robO3);
rob1=min([rob11 rob12 rob13]);
%
for i=t(1):t(2)
    robT(i-t(1)+1)=min([C(13,:)*x(:,i)+c(13) C(14,:)*x(:,i)+c(14) C(15,:)*x(:,i)+c(15) C(16,:)*x(:,i)+c(16)]);
end
rob2=max(robT);
for i=g(1):g(2)
    robG(i-g(1)+1)=min([C(17,:)*x(:,i)+c(17) C(18,:)*x(:,i)+c(18) C(19,:)*x(:,i)+c(19) C(20,:)*x(:,i)+c(20)]);
end
rob3=max(robG);
rob_ind=min([rob1 rob2 rob3]);
%
for i=j(1,1):j(1,2)
    rob_1(i-j(1,1)+1)=-norm(x(:,i)-xk1(:,i),1)+e1;
end
for i=j(2,1):j(2,2)
    rob_2(i-j(2,1)+1)=-norm(x(:,i)-xk2(:,i),1)+e2;
end
rob_joint=min([max(rob_1) max(rob_2)]);
%
rob=min(rob_ind,rob_joint);
%
end
%
% robustness for agents with three edges
function [rob, rob_ind, rob_joint] = compute_robustness_three_edges(x,xk1,xk2,xk3,C,c,e1,e2,e3,t,g,j,N)
%
robO1=[]; robO2=[]; robO3=[]; robO=[];
robT=[];
robG=[];
rob_1=[];
rob_2=[];
rob_3=[];
for i=1:N+1
    robO1(i)=min([C(1,:)*x(:,i)+c(1) C(2,:)*x(:,i)+c(2) C(3,:)*x(:,i)+c(3) C(4,:)*x(:,i)+c(4)]);
    robO2(i)=min([C(5,:)*x(:,i)+c(5) C(6,:)*x(:,i)+c(6) C(7,:)*x(:,i)+c(7) C(8,:)*x(:,i)+c(8)]);
    robO3(i)=min([C(9,:)*x(:,i)+c(9) C(10,:)*x(:,i)+c(10) C(11,:)*x(:,i)+c(11) C(12,:)*x(:,i)+c(12)]);
end
rob11=-max(robO1); rob12=-max(robO2); rob13=-max(robO3);
rob1=min([rob11 rob12 rob13]);
%
for i=t(1):t(2)
    robT(i-t(1)+1)=min([C(13,:)*x(:,i)+c(13) C(14,:)*x(:,i)+c(14) C(15,:)*x(:,i)+c(15) C(16,:)*x(:,i)+c(16)]);
end
rob2=max(robT);
for i=g(1):g(2)
    robG(i-g(1)+1)=min([C(17,:)*x(:,i)+c(17) C(18,:)*x(:,i)+c(18) C(19,:)*x(:,i)+c(19) C(20,:)*x(:,i)+c(20)]);
end
rob3=max(robG);
rob_ind=min([rob1 rob2 rob3]);
%
for i=j(1,1):j(1,2)
    rob_1(i-j(1,1)+1)=-norm(x(:,i)-xk1(:,i),1)+e1;
end
for i=j(2,1):j(2,2)
    rob_2(i-j(2,1)+1)=-norm(x(:,i)-xk2(:,i),1)+e2;
end
for i=j(3,1):j(3,2)
    rob_3(i-j(3,1)+1)=-norm(x(:,i)-xk3(:,i),1)+e3;
end
rob_joint=min([max(rob_1) max(rob_2) max(rob_3)]);
%
rob=min(rob_ind,rob_joint);
%
end

