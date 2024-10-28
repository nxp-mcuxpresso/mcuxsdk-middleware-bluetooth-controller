function [ResRaw,rssi,agcidx] = RttReadLog(file_path)
if nargin < 1
    file_path = fullfile('Captures','rtt_dbg_initiator.txt');
end
fid = fopen(file_path,'r');
txt_raw = textscan(fid,'%s','delimiter','\n');
fclose(fid);
for m = 1:length(txt_raw{1})
    if contains(txt_raw{1}(m),'rssi')
        temp = txt_raw{1}(m);
        temp = temp{1};
        idx_rssi0_start = strfind(temp,'rssi[0]=') + 8;
        idx_rssi0_end = strfind(temp,'rssi[1]=')-2;
        idx_rssi1_start = strfind(temp,'rssi[1]=') + 8;
        rssi(1) = str2double(temp(idx_rssi0_start:idx_rssi0_end));
        rssi(2) = str2double(temp(idx_rssi1_start:end));
    end
    if contains(txt_raw{1}(m),'agc_idx')
        temp = txt_raw{1}(m);
        temp = temp{1};
        idx_agcidx_start = strfind(temp,'agc_idx=') + 8;
        agcidx = str2double(temp(idx_agcidx_start:end));
    end
    if contains(txt_raw{1}(m),'stepno')
        header = txt_raw{1}(m);
        header = str2cell(header{1});
        idx_start = m+1;
    end
end

ResRaw = struct;
for m = (idx_start+2):length(txt_raw{1})
    temp_raw = txt_raw{1}(m);
    temp_raw = str2cell(temp_raw{1});
    for k1 = 1:length(temp_raw)
        if contains(temp_raw{k1},'0x')
            temp = strrep(temp_raw{k1},'0x','');
        elseif contains(temp_raw{k1},'0X')
            temp = strrep(temp_raw{k1},'0X','');
        else
            temp = str2double(temp_raw{k1});
        end
        eval(sprintf('ResRaw(m-(idx_start+1),:).%s = temp;', header{k1}))
    end
end

function out = str2cell(in)
comma_idx = [0,strfind(in,','),length(in)+1];
out = cell(length(comma_idx)-1,1);
for m = 1:(length(comma_idx)-1)
    out{m} = in(comma_idx(m)+1 : comma_idx(m+1)-1);
end