function TF = inputyn(inputstr)
% inputYN is a wrapper of input. Ask question with Y or N for the answer to
% get true or false output, respectively. To interrupt, if you type
% 'keyboard' in response you can enter debugging mode.
% 
%   TF = inputYN(inputstr)
% 
%  inputstr     String for a question. ' (Y/N):' will be added at the end.
%
%  TF           A logical output. true is for Y or y, false is for N or n 
%
% 
% EXAMPLE
% if inputYN('Do yo want to proceed?')
%     dsip('OK')
% else
%     dsip('cancelled') 
% end
%     
% % the above code prints the following in the command window
% Do yo want to proceed? (Y/N): 
%
% See also
% input
%
% Written by
% Kouichi.C.Nakamura, Ph.D.
% Kyoto University
% kouichi.c.nakamura@gmail.com
% 16 Nov 2015

while 1
    strResponse = input([inputstr,' (Y/N):'],'s');
    if strcmpi('Y', strResponse)
        TF = true;
        break;
    elseif strcmpi('N', strResponse)
        TF = false;
        break;
    elseif strcmp('keyboard', strResponse)
        keyboard
        break;
    end
end


end
