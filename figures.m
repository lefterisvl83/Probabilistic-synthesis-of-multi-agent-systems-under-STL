%
figure(1);
X.plot('alpha',0,'LineStyle','--'); hold on;
Xtight.plot('alpha',0.2,'color','lightgray')
O1.plot('color','red','alpha',1); hold on;
O2.plot('color', 'red','alpha',1);
O3.plot('color', 'red','alpha',1);
T1.plot('color',[0 0.5 0.5],'alpha',0.2); G1.plot('color',[0 0.5 0.5],'alpha',0.2);     % teal color
T2.plot('color',[0 1 0],'alpha',0.2); G2.plot('color',[0 1 0],'alpha',0.2);             % green color
T3.plot('color',[0 0 1],'alpha',0.2); G3.plot('color',[0 0 1],'alpha',0.2);             % blue color
T4.plot('color',[0 1 1],'alpha',0.2); G4.plot('color',[0 1 1],'alpha',0.2);             % cyan color
T5.plot('color', [1 0 1],'alpha',0.2); G5.plot('color', [1 0 1],'alpha',0.2);           % magenta color
T6.plot('color',[1 1 0],'alpha',0.2); G6.plot('color',[1 1 0],'alpha',0.2);             % yellow color
T7.plot('color',[1 0.5 0],'alpha',0.2); G7.plot('color',[1 0.5 0],'alpha',0.2);         % orange color
T8.plot('color',[0.5 0 0.5],'alpha',0.2); G8.plot('color',[0.5 0 0.5],'alpha',0.2);     % purple color
T9.plot('color',[1 0.8 0.8],'alpha',0.2); G9.plot('color',[1 0.8 0.8],'alpha',0.2);     % pink color
T10.plot('color',[0.64 0.16 0.16],'alpha',0.2); G10.plot('color',[0.64 0.16 0.16],'alpha',0.2); % brown color
axis([-1 50 -3 22]);
title('Workspace','FontSize',16,'Interpreter','Latex')
hold off;
%
% tight polyhedra
Ot1=Polyhedron(O1.A,O1.b+tighteningO1);
Ot2=Polyhedron(O2.A,O2.b+tighteningO2);
Ot3=Polyhedron(O3.A,O3.b+tighteningO3);
Tt1=Polyhedron(T1.A,T1.b+tighteningT1); Gt1=Polyhedron(G1.A,G1.b+tighteningG1);
Tt2=Polyhedron(T2.A,T2.b+tighteningT2); Gt2=Polyhedron(G2.A,G2.b+tighteningG2);
Tt3=Polyhedron(T3.A,T3.b+tighteningT3); Gt3=Polyhedron(G3.A,G3.b+tighteningG3);
Tt4=Polyhedron(T4.A,T4.b+tighteningT4); Gt4=Polyhedron(G4.A,G4.b+tighteningG4);
Tt5=Polyhedron(T5.A,T5.b+tighteningT5); Gt5=Polyhedron(G5.A,G5.b+tighteningG5);
Tt6=Polyhedron(T6.A,T6.b+tighteningT6); Gt6=Polyhedron(G6.A,G6.b+tighteningG6);
Tt7=Polyhedron(T7.A,T7.b+tighteningT7); Gt7=Polyhedron(G7.A,G7.b+tighteningG7);
Tt8=Polyhedron(T8.A,T8.b+tighteningT8); Gt8=Polyhedron(G8.A,G8.b+tighteningG8);
Tt9=Polyhedron(T9.A,T9.b+tighteningT9); Gt9=Polyhedron(G9.A,G9.b+tighteningG9);
Tt10=Polyhedron(T10.A,T10.b+tighteningT10); Gt10=Polyhedron(G10.A,G10.b+tighteningG10);
%
%
hold on;
Ot1.plot('color','red','alpha',.2);
Ot2.plot('color', 'red','alpha',.2);
Ot3.plot('color', 'red','alpha',0.2);
Tt1.plot('color',[0 0.5 0.5],'alpha',1); Gt1.plot('color',[0 0.5 0.5],'alpha',1);     % teal color
Tt2.plot('color',[0 1 0],'alpha',1); Gt2.plot('color',[0 1 0],'alpha',1);             % green color
Tt3.plot('color',[0 0 1],'alpha',1); Gt3.plot('color',[0 0 1],'alpha',1);             % blue color
Tt4.plot('color',[0 1 1],'alpha',1); Gt4.plot('color',[0 1 1],'alpha',1);             % cyan color
Tt5.plot('color', [1 0 1],'alpha',1); Gt5.plot('color', [1 0 1],'alpha',1);           % magenta color
Tt6.plot('color',[1 1 0],'alpha',1); Gt6.plot('color',[1 1 0],'alpha',1);             % yellow color
Tt7.plot('color',[1 0.5 0],'alpha',1); Gt7.plot('color',[1 0.5 0],'alpha',1);         % orange color
Tt8.plot('color',[0.5 0 0.5],'alpha',1); Gt8.plot('color',[0.5 0 0.5],'alpha',1);     % purple color
Tt9.plot('color',[1 0.8 0.8],'alpha',1); Gt9.plot('color',[1 0.8 0.8],'alpha',1);     % pink color
Tt10.plot('color',[0.64 0.16 0.16],'alpha',1); Gt10.plot('color',[0.64 0.16 0.16],'alpha',1); % brown color
%
%
x1=xk1{end};x2=xk2{end};x3=xk3{end};x4=xk4{end};x5=xk5{end};
x6=xk6{end};x7=xk7{end};x8=xk8{end};x9=xk9{end};x10=xk10{end};
v1=uk1{end};v2=uk2{end};v3=uk3{end};v4=uk4{end};v5=uk5{end};
v6=uk6{end};v7=uk7{end};v8=uk8{end};v9=uk9{end};v10=uk10{end};
rng(3);
num_samples=1e5;
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

figure(1); hold on;
for i=1:num_samples
    plot(xk{i}{1}(1,:),xk{i}{1}(2,:),'-','linewidth',1., 'Color',[0 0.5 0.5])
end
for i=1:num_samples
    plot(xk{i}{2}(1,:),xk{i}{2}(2,:),'-','linewidth',1., 'Color',[0 1 0])
end
for i=1:num_samples    
    plot(xk{i}{3}(1,:),xk{i}{3}(2,:),'-','linewidth',1., 'Color',[0 0 1])
end
for i=1:num_samples    
    plot(xk{i}{4}(1,:),xk{i}{4}(2,:),'-','linewidth',1., 'Color',[0 1 1])
end
for i=1:num_samples    
    plot(xk{i}{5}(1,:),xk{i}{5}(2,:),'-','linewidth',1., 'Color',[1 0 1])
end
for i=1:num_samples    
    plot(xk{i}{6}(1,:),xk{i}{6}(2,:),'-','linewidth',1., 'Color',[1 1 0])
end
for i=1:num_samples    
    plot(xk{i}{7}(1,:),xk{i}{7}(2,:),'-','linewidth',1., 'Color',[1 0.5 0])
end
for i=1:num_samples    
    plot(xk{i}{8}(1,:),xk{i}{8}(2,:),'-','linewidth',1., 'Color',[0.5 0 0.5])
end
for i=1:num_samples    
    plot(xk{i}{9}(1,:),xk{i}{9}(2,:),'-','linewidth',1., 'Color',[1 0.8 0.8])
end
for i=1:num_samples    
    plot(xk{i}{10}(1,:),xk{i}{10}(2,:),'-','linewidth',1., 'Color',[0.64 0.16 0.16])
end
%
centO1=mean(O1.V',2);
text(centO1(1),centO1(2), '$O_1$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centO2=mean(O2.V',2);
text(centO2(1),centO2(2), '$O_2$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centO3=mean(O3.V',2);
text(centO3(1),centO3(2), '$O_3$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
% region T
centT1=mean(T1.V'-[.5;2.1],2);
text(centT1(1),centT1(2), '$T_1$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT2=mean(T2.V'-[.5;2.1],2);
text(centT2(1),centT2(2), '$T_2$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT3=mean(T3.V'-[.5;2.1],2);
text(centT3(1),centT3(2), '$T_3$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT4=mean(T4.V'-[.5;2.1],2);
text(centT4(1),centT4(2), '$T_4$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT5=mean(T5.V'-[.5;2.1],2);
text(centT5(1),centT5(2), '$T_5$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT6=mean(T6.V'-[.5;2.1],2);
text(centT6(1),centT6(2), '$T_6$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT7=mean(T7.V'-[.5;2.1],2);
text(centT7(1),centT7(2), '$T_7$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT8=mean(T8.V'-[.5;2.1],2);
text(centT8(1),centT8(2), '$T_8$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT9=mean(T9.V'-[.5;2.1],2);
text(centT9(1),centT9(2), '$T_9$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centT10=mean(T10.V'-[.5;2.1],2);
text(centT10(1),centT10(2), '$T_{10}$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
% region G
centG1=mean(G1.V'+[.0;1.7],2);
text(centG1(1),centG1(2), '$G_1$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG2=mean(G2.V'+[.0;1.7],2);
text(centG2(1),centG2(2), '$G_2$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG3=mean(G3.V'+[.0;1.7],2);
text(centG3(1),centG3(2), '$G_3$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG4=mean(G4.V'+[.0;1.7],2);
text(centG4(1),centG4(2), '$G_4$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG5=mean(G5.V'+[.0;1.7],2);
text(centG5(1),centG5(2), '$G_5$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG6=mean(G6.V'+[.0;1.7],2);
text(centG6(1),centG6(2), '$G_6$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG7=mean(G7.V'+[.0;1.7],2);
text(centG7(1),centG7(2), '$G_7$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG8=mean(G8.V'+[.0;1.7],2);
text(centG8(1),centG8(2), '$G_8$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG9=mean(G9.V'+[.0;1.7],2);
text(centG9(1),centG9(2), '$G_9$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');
centG10=mean(G10.V'+[.0;1.7],2);
text(centG10(1),centG10(2),'$G_{10}$', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k','Interpreter','Latex');





figure(2); hold on;
plot([rob1{:}],'--x','Linewidth',1.5,'Color',[0 0.5 0.5]);
plot([rob2{:}],'-*','Linewidth',1.5,'Color',[0 1 0]);
plot([rob3{:}],'-o','Linewidth',1.5,'Color',[0 0 1]);
plot([rob4{:}],'-*','Linewidth',1.5,'Color',[0 1 1]);plot([rob5{:}],'-*','Linewidth',1.5,'Color',[1 0 1]);
plot([rob6{:}],'-o','Linewidth',1.5,'Color',[1 1 0]);
plot([rob7{:}],'--x','Linewidth',1.5,'Color',[1 0.5 0]);plot([rob8{:}],'-o','Linewidth',1.5,'Color',[0.5 0 0.5]);
plot([rob9{:}],'-*','Linewidth',1.5,'Color',[1 0.8 0.8]);
plot([rob10{:}],'--X','Linewidth',1.5,'Color',[0.64 0.16 0.16]);
grid on; 
xlabel('Iterations','FontSize',16,'Interpreter','Latex')
ylabel('$\rho^{\phi_i}$','FontSize',16,'Interpreter','Latex')
hold off;
legend('Agent-1','Agent-2','Agent-3','Agent-4','Agent-5','Agent-6', ...
    'Agent-7','Agent-8','Agent-9','Agent-10','Fontsize',10,'Interpreter','Latex')
















