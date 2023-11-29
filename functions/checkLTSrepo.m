function checkLTSrepo
    %CHECKLTSREPO checks if LTS class exists or rather is in the search
    %path, if not it try to clone the repo from git
    %
    %the LTS class control Thorlabs long travel stages (LTS) via MATLAB by 
    %using the Thorlabs .Net DLLs from Kinesis Software
    %
    %note Thorlabs's Kinesis Software is required: 
    % https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0
    
    className = 'LTS';

    try
        classInfo = meta.class.fromName(className);
        if any(strcmp({classInfo.MethodList.Name}, 'movetopos'))
            disp(['class ' className ' found. perfect.']);
             warning('note Thorlabs Kinesis Software is required: https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0 ')
        else
            getLTSrepo()
        end
    catch
        disp(['class ' className ' not found. try to clone from git...']);
        getLTSrepo()
    end
end

