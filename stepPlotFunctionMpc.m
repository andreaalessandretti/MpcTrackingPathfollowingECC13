

function h = stepPlotFunctionMpc(agentsList,hist,plot_handles,i,dt,tt)


if not(plot_handles == 0)
    delete(plot_handles)
end


%% Plot path
pd = agentsList{1}.controller.mpcOp.auxiliaryLaw.pd;
t = i*dt;

T = agentsList{1}.controller.mpcOp.horizonLength;

x = hist{1}.stateTrajectory(:,1:i-1); hold on;

x_opt = agentsList{1}.controller.lastSolution.x_opt;

if tt
    pdAll = pd(0:0.1:t+T);
    pdt   = pd(t);
else
    
    pdAll = pd(x(end,:));
    pdt   = pd(x_opt(end,1));
    
end

h(1) = plot(pdAll(1,:),pdAll(2,:),'r--'); hold on 
h(2) = plot(pdt(1),pdt(2),'ro');

%% Plot state trajectory
h(3) = plot(x(1,:),x(2,:));

%% Plot prediction

h(4) = plot(x_opt(1,:),x_opt(2,:),'--');hold on;

grid on

end
