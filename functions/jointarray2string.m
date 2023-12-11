function jointstring = jointarray2string(jointarray)
    %JOINTARRAY2STRING convert a nx6 array with angle joint values to a nx1 
    % string with the angle values in the form 'J(x,x,x,x,x,x)'

    if size(jointarray,2) ~= 6
        error('expected input of cobotta angles is 6')
    end
    
    if size(jointarray,1) == 1
        jointstring = "J(";
        for angle = 1:6
            jointstring = strcat(jointstring,num2str(jointarray(angle)));
            if angle ~= 6 
                jointstring = strcat(jointstring,',');
            else
                jointstring = strcat(jointstring, ')');
            end
        end
    else
        jointstring = [];
        for input= 1:size(jointarray,1)
            for angle = 1:6
                if angle == 1
                    string = "J(";
                end
                string = strcat(string,num2str(jointarray(angle)));
                if angle ~= 6 
                    string = strcat(string,',');
                else
                    string = strcat(string, ')');
                end
            end
            jointstring = [jointstring;string];
        end
    end
         
end

