%%dh parameter
%%GUI
%create a robot class for hans_cute

%%code main 

%q1=getpos(current arm)

%(thickness,placement) = read_book

%q2 for arm = book_chute
%q3 for arm = book_placement 

%grip_open = thickness_book+offset

%arm_action = (q1,q2)
%grip_close

%arm_action = (q2,q3)
%grip_open = thickness_book+offset






%function = arm_action (q_start,q_des)
    %trajectory calculating = trapezoidal(q_start,q_des)
    %collision_check
    
    %until trajectory is collision free
    
    %start looping + animate
        %animate step
        %emergency_check
        %collision_check
    
    %end loop
%end


%function = collision_check(qin)
    %fkine.(qin)
%end


%function = emergency_check(button)
    %if (button = true)
%end

%function = read_book(code)
    %book_thickness
    %book_placement
%end

%function = grip_action








