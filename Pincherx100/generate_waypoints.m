function waypoints=generate_waypoints(start,goal,num_waypoints)
    waypoints=zeros(num_waypoints,length(start));
    for i=1:length(start)
        waypoints(:,i)=linspace(start(i),goal(i),num_waypoints);
    end

    %disp(waypoints(1:5));
end