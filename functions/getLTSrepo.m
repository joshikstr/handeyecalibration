function getLTSrepo
    %GETLTSREPO clone the git repositoy driverThorlabsLTS 
    %(require MATLAB R2023b or higher)
    
    if ~isMATLABReleaseOlderThan('R2023b')
        gitclone('https://github.com/joshikstr/driverThorlabsLTS','functions/driverThorlabsLTS');
        disp('...done')
        warning('note Thorlabs Kinesis Software is required: https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0')
        addpath('functions\driverThorlabsLTS')
    else
        error_message = ['git clone failed (require MATLAB R2023b or higher) \n' ,...
            'please manually clone the repository from URL: https://github.com/joshikstr/driverThorlabsLTS and add it to the path'];
        error('u:stuffed:it', error_message);
    end
end

