function RttRes = RttUnpackRes(RttResRaw)

RttRes = struct;
for m = 1:length(RttResRaw)
    % Timestamp 1
    if isfield(RttResRaw(m),'t1')
        RttRes(m).T1 = RttResRaw(m).t1; % T1 is ToD on initiator, ToA on reflector
    else
        RttRes(m).T1 = 0;
    end
    % Timestamp 2
    if isfield(RttResRaw(m),'t2')
        RttRes(m).T2 = RttResRaw(m).t2; % T2 is ToA on initiator, ToD on reflector
    else
        RttRes(m).T2 = 0;
    end
    % PN pattern
    if isfield(RttResRaw(m),'AA')
        RttRes(m).pn = RttResRaw(m).AA;
    else
        RttRes(m).pn = '6B7D9171';
    end
    % HARTT status register
    if isfield(RttResRaw(m),'hartt_stat')
        temp = RttResRaw(m).hartt_stat;
    else
        temp = 0;
    end
    % PHY status register0 (contains CFO and FRAC from AA detector)
    if isfield(RttResRaw(m),'stat0')
        temp0 = RttResRaw(m).stat0;
    else
        temp0 = 0;
    end
    
    RttRes(m).rtt_res_raw = temp;
    % Extract the bit-fields
    RttRes(m).rtt_cfo = bitand(temp,hex2dec('FFFF'));
    RttRes(m).rtt_p_delta = bitshift(bitand(temp,hex2dec('3FF0000')),-16);
    RttRes(m).rtt_ham_dist_sat = bitshift(bitand(temp,hex2dec('C000000')),-26);
    RttRes(m).rtt_int_adj = bitshift(bitand(temp,hex2dec('30000000')),-28);
    RttRes(m).rtt_found = bitshift(bitand(temp,hex2dec('40000000')),-30);
    
    % Convert to actual numbers
    RttRes(m).rtt_cfo = double(reinterpretcast(uint16(RttRes(m).rtt_cfo),numerictype(1,16,15)));
    RttRes(m).rtt_int_adj = double(fi(RttRes(m).rtt_int_adj,1,2,0,'OverflowAction','Wrap'));
    RttRes(m).rtt_p_delta = double(reinterpretcast(fi(RttRes(m).rtt_p_delta,0,10,0),numerictype(1,10,9)));
    
    % Legacy CFO/frac
    RttRes(m).leg_cfo = bitshift(bitand(temp0,hex2dec('03FF0000')),-16);
    RttRes(m).leg_frac = bitshift(bitand(temp0,hex2dec('00003F00')),-8);
    RttRes(m).leg_cfo = double(reinterpretcast(fi(RttRes(m).leg_cfo,0,10,0),numerictype(1,10,9)));
    RttRes(m).leg_frac = double(reinterpretcast(fi(RttRes(m).leg_frac,0,6,0),numerictype(1,6,5)));
end
