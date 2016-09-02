%%  
% reads master log file from orca and plots the intervals between
% timestamps of each type of data. 
% Useful to quickly check if what should be X Hz is really X Hz.
%
%% 
if ( ~exist('masterlogfile') )
    'YOU MUST DEFINE A VARIABLE NAMED masterlogfile WITH THE FILE NAME OF THE MASTER LOG FILE'
    break;
end
    
fid = fopen(masterlogfile);

got_data = 0; %% have we found the #data line?
num_logfiles = 0; %% how many kinds of data we have?

% go up to the end of the header, or the end of file
while (~got_data) && ( ~feof(fid))
	
	C = textscan(fid,'%s',1,'delimiter','\n'); % get a line	
	if (~strcmp('#',C{1}{1}(1))) 
		% if it is not a coment, it indicates a log file.
		num_logfiles=num_logfiles+1;
		logfilenames{num_logfiles}=C{1}{1}; % get its name
	end
	got_data = strcmp(C{1},'#data'); % we have found the data!
end


D = textscan(fid,'%u %u %u %u','delimiter','\n'); % read everything. 4 cols	
% if the last line is broken, the first collumn will have 1 line more than
% the last. in this case we should eliminate the last broken line
s1 = length(D{1});s2 = length(D{2});s3 = length(D{3});s4 = length(D{4});
minl = min([s1 s2 s3 s4]); maxl = max([s1 s2 s3 s4])
if ((maxl - minl)>1) 
    msg = sprintf('WARNING: master file may be broken, its collumns have different lengths')
elseif ((maxl - minl)>0)
    msg = sprintf('master file: last line is broken, will ignore it')    
end

D = double([D{1}(1:minl) D{2}(1:minl) D{3}(1:minl) D{4}(1:minl)]); 
D = [D(:,1)+(D(:,2)/1000000) D(:,3:4)]; % calculate timestamps. now 3 cols

!rm periods.pdf
!rm periods.ps

for l = 1:(num_logfiles) % for each logfile
	ts = D((D(:,2)==(l-1)),1); % get timestamps concernign the logfile
    if (length(ts)<=0)
        msg = sprintf('WARNING: no data for %s',logfilenames{l})
        continue;
    end
	dt = ts(2:end)-ts(1:end-1); % get the interval between them
	
	dt_mean(l) = mean(dt);% mean and std
	dt_std(l) = std(dt);
	   
	% and plot! 
	figure; f = gcf; p = get(f,'Position');
    set(f,'Position',p .* [1 1 2 1]);% double the width - 2 subplots
    p = get(f,'PaperPosition');s = get(f,'PaperSize');
    set(f,'PaperPosition',p .* [1 1 2 1]); % double the width - 2 subplots
    p = get(f,'PaperPosition');
    set(f,'PaperPosition',p .* [1 1 [1 1]*s(2)/p(3)]); % make paper width be sheet height
    set(f,'PaperPositionMode','manual'); % honor PaperPosition, not the same size as on screen
    set(f,'PaperOrientation','landscape');
    subplot(1,2,1); hold on; grid on;
	plot(ts(2:end)-ts(1),dt*1000,'+'); % subtraction: time is relative to the first one.
    % lines for mean, mean+-std, mean+-2std
    a = gca; xl = get(a,'XLim');
    line([xl(1) xl(2)],1000*[dt_mean(l) dt_mean(l)],'Color','b');
    line([xl(1) xl(2)],1000*([dt_mean(l) dt_mean(l)]+dt_std(l)),'Color','g','LineStyle','-.');
    line([xl(1) xl(2)],1000*([dt_mean(l) dt_mean(l)]-dt_std(l)),'Color','g','LineStyle','-.');
    line([xl(1) xl(2)],1000*([dt_mean(l) dt_mean(l)]+2*dt_std(l)),'Color','r','LineStyle','-.');
    line([xl(1) xl(2)],1000*([dt_mean(l) dt_mean(l)]-2*dt_std(l)),'Color','r','LineStyle','-.');

	xlabel('time (s)','FontSize',16);
	ylabel('interval (ms)','FontSize',16);
    title(logfilenames{l},'FontSize',16,'Interpreter','none'); 
    msg1=sprintf('freq(Hz):%f std:%f',mean(1./dt),std(1./dt));
    msg2=sprintf('per. mean %f (ms)',1000*dt_mean(l));
    msg3=sprintf('per.  std %f (ms)',1000*dt_std(l));
    legend(msg1,msg2,msg3,'Location','Best');
    hold off;
    
  	% and plot histogram too! 
	subplot(1,2,2); hold on; grid on;
	hist(dt*1000,100);
	xlabel('interval (ms)','FontSize',16);
    title('Histogram','FontSize',16); 
	hold off; 
    print -dpsc2 -append periods.ps	   

end

!ps2pdf periods.ps periods.pdf

fclose(fid);