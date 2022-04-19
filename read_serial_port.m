clear;
%s=serial('/dev/cu.wchusbserialfa130','BaudRate',115200);
s=serial('/dev/cu.wchusbserialfd120','BaudRate',115200);
%s=serial('/dev/cu.HC-05-DevB','BaudRate',115200);

c=0;
fopen(s);
A=[];
f1=figure;
h = animatedline;
axis([0 3000 0 1000]);
%axis([-2500 2500 -4600 4600]);
while ishandle(f1)
    %a=fscanf(s,'%f: vr/vl: %f/%f\tx y theta,en/ir/filter: %f/%f/%f\t%f\t%f/%f/%f\n');
    %a=fscanf(s,'%d/%d\t%f: x y v a: %f\t%f\t%f/%f/%f\t%f\n');	
    %a=fscanf(s,'%d/%d\t%f: wr/wl: %d/%d\t%d/%d\tx y theta,en/ir/filter: %f/%f/%f\t%f\t%f/%f/%f\n');
    %'h':
    %a=fscanf(s,'%f: x y v a: %f/%f/%f\t%f\t%f/%f/%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n');
    %'t':
    %a=fscanf(s,'%f: x y v a: %f/%f/%f/%f\t%f\t%f/%f/%f\t%f\t%f/%f\t%f/%f\t%f\n');
    %'v':
    %a=fscanf(s,'%f: x y theta v: %f\t%f\t%f\t%f/%f/%f/%f\n');
    %o:
    %a=fscanf(s,'%f: x y v a: %f/%f/%f/%f\t%f/%f\t%f/%f\t%f\t%f\t%f\t%f\n');
    %'x'
    %a=fscanf(s,'%f: %f/%f/%f/%f\t%f/%f\t%f\t%f\t%f\t%f\t%f/%f/%f\t%f\n');
    %'c'
    a=fscanf(s,'%f: %f/%f/%f/%f\t%f/%f\t%f\t%f\t%f/%f\t%f\t%f/%f/%f\t%f/%f/%f\n');
    A=[A,a];
    %t:
    %addpoints(h,a(6),a(10));
    %addpoints(h,a(3),a(5));
    %h:
    %addpoints(h,a(15),a(6));
    %o/x/c:
    addpoints(h,a(6),a(2));
    drawnow
end
fclose(instrfind);
dt=A(1,10)-A(1,9);