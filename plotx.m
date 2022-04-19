input='o';
if input=='x'
    subplot(2,1,1);
    plot(A(6,:),A(2,:),A(7,:),A(3,:));

    subplot(2,1,2);
    plot(A(8,:),A(9,:),A(8,:),A(10,:),A(8,:),A(11,:));
    legend("track","v","stbase");
    
    subplot(2,1,3);
    plot(A(8,:),A(12,:),A(8,:),A(13,:),A(8,:),A(14,:));
    legend("p","i","d");
end
if input=='c'
    subplot(3,1,1);
    plot(A(6,:),A(2,:),A(7,:),A(3,:));

    subplot(3,1,2);
    plot(A(8,:),A(9,:),A(8,:),A(10,:),A(8,:),A(11,:),A(8,:),A(13,:),A(8,:),A(14,:),A(8,:),A(15,:));
    legend("track","v","v need","brbase","fanbase","v base");
    
    subplot(3,1,2);
    plot(A(8,:),A(12,:),A(8,:),A(16,:),A(8,:),A(17,:),A(8,:),A(18,:));
    legend("brake","p","i","d");
    
end
if input=='h'
    subplot(2,1,1);
    %plot(A(5,:),A(8,:),y,v)
    plot(A(5,:),A(6,:),A(5,:),A(7,:));
    hold on
    plot(A(5,:),A(12,:)*2.5);
    plot(A(5,:),A(13,:)-950);

    subplot(2,1,2);
    plot(A(5,:),-A(8,:),A(5,:),A(9,:),A(5,:),A(10,:),A(5,:),A(11,:));
    legend("a","error","integral","brake");
    
end
if input=='o'
    plot(A(6,:),A(2,:),A(7,:),A(3,:),A(7,:),A(4,:),A(7,:),A(5,:)+2000);
    title("x");
    legend("x_dr","x","x_en","x_ir");
end
if input=='t'
    subplot(4,1,1);
    plot(A(6,:),A(2,:),A(6,:),A(3,:),A(6,:),A(4,:),A(6,:),A(5,:)+2000);
    title("x");
    legend("x","en","en avg","ir");
    subplot(4,1,3);
    plot(A(6,:),A(10,:));
    title("v");
    subplot(4,1,2);
    plot(A(6,:),A(9,:),A(6,:),A(7,:),A(6,:),A(8,:));
    legend("theta","en","ir")
    title("theta");
    subplot(4,1,4);
    plot(A(6,:),A(13,:)-A(14,:));
    title("delta count");
end