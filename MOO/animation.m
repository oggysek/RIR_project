%% Simulink 3D Animation
animSpeed = 0.45;

figure;
plot3(WayPts(:,1),WayPts(:,2),WayPts(:,3), "bo")    % targets
xlimit = [-1 4];
ylimit = [-1.5 3.5];
zlimit = [-12 0];
width = 750;
height = 650;
NewFigure(xlimit,ylimit,zlimit,-43,25,width,height);
pause(1)
AnimEulerTar(out.time,out.XYZ,out.EulerAngles,out.VXYZ,out.Tar,animSpeed)


%% Local Functions

function NewFigure(xlim,ylim,zlim,viewx,viewy,w,h)
    set(gca, 'XLim', xlim,'YLim',ylim,'ZLim',zlim,'ZDir','reverse');
    view(viewx,viewy)
    x0=10;
    y0=10;
    set(gcf,'position',[x0,y0,w,h])
    hold on;
    grid on;
end

function AnimEulerTar(t_plot,XYZs,EulerAngles,VXYZs,Tars,animSpeed)
    t_section = 0;
    curve = animatedline('LineWidth',0.5);
    curveTR = animatedline('LineWidth',1,'LineStyle',':');
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Do Animation
            Euler = EulerAngles(i,:);
            XYZ = XYZs(i,:);
            VXYZ = VXYZs(i,:);
            TR = Tars(i,:);

            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O;

            pro1 = O_I(:,1)+O_I(:,2) + transpose(XYZ);           
            pro2 = O_I(:,1)-O_I(:,2) + transpose(XYZ);            
            pro3 = -O_I(:,1)+O_I(:,2) + transpose(XYZ);           
            pro4 = -O_I(:,1)-O_I(:,2) + transpose(XYZ);

            addpoints(curve, XYZ(1), XYZ(2),XYZ(3))
            addpoints(curveTR, TR(1),TR(2),TR(3))
            head = scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black');

            line1 = drawline(XYZ,O_I(:,1),'b--',1.5);
            line2 = drawline(XYZ,O_I(:,2),'g--',1.5);
            line3 = drawline(XYZ,O_I(:,3),'r--',1.5);
            line5 = extendline(XYZ,O_I(:,1),'b:');

            frame1 = drawline(XYZ,0.5*O_I(:,1)+0.5*O_I(:,2),'black',2.5);
            frame2 = drawline(XYZ,0.5*O_I(:,1)-0.5*O_I(:,2),'black',2.5);
            frame3 = drawline(XYZ,-0.5*O_I(:,1)+0.5*O_I(:,2),'black',2.5);
            frame4 = drawline(XYZ,-0.5*O_I(:,1)-0.5*O_I(:,2),'black',2.5);           

            drawnow
            pause(0.01)

            % logs     
            xlabel(string( num2str(t_plot(i),'%.1f') )+' sec   ');

            dispstr7 = string( num2str( VXYZ(1),'%.1f' ) );
            dispstr8 = string( num2str( VXYZ(2),'%.1f' ) );
            dispstr9 = string( num2str( VXYZ(3),'%.1f' ) );
            vstr = 'Velocity ['+dispstr7+ ' , '+dispstr8 + ' , ' + dispstr9 + ']';

            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );
            title('EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
                + ' , ' +dispstr5+ ' , '+dispstr6 +']   ' + vstr);

            t_section = t_section + animSpeed;            

            delete(line1)
            delete(line2)
            delete(line3)
            delete(line5)

            delete(frame1)
            delete(frame2)
            delete(frame3)
            delete(frame4)

            delete(head)

        end
    end    
end

function m = matrixB2I(phi,theta,psi)
    T_BtoV2 = [[1 0 0];[0 cos(-phi) sin(-phi)];[0 -sin(-phi) cos(-phi)]];
    T_V2toV1 = [[cos(-theta) 0 -sin(-theta)];[0 1 0];[sin(-theta) 0 cos(-theta)]];
    T_V1toI = [[cos(-psi) sin(-psi) 0];[-sin(-psi) cos(-psi) 0];[0 0 1]];
    m = T_V1toI*T_V2toV1*T_BtoV2;
end

function line = drawline(p1,p2,color,width)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1;
    pt2 = pt1 + transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',width);
end

function line = extendline(p1,p2,color)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1;
    pt2 = pt1 + 20*transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',0.5);
end

function VisAttitude(Euler,linsty)
    O = eye(3);
    T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
    O_I = T_BtoI*O;
    for i = 1:length(O_I)
        drawline(O_I(:,i),linsty)
    end
    z = O_I(:,3);
    scatter3(z(1),z(2),z(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b')
end


