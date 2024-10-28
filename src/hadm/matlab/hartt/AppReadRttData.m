clear all;
%uc_idx_vec = 2:11;
uc_idx_vec = [2 3 4 7 8 10];
Rssi0VecI = zeros(size(uc_idx_vec));
Rssi1VecI = zeros(size(uc_idx_vec));
AgcIdxVecI = zeros(size(uc_idx_vec));
Rssi0VecR = zeros(size(uc_idx_vec));
Rssi1VecR = zeros(size(uc_idx_vec));
AgcIdxVecR = zeros(size(uc_idx_vec));
for uc_idx_m = 1:length(uc_idx_vec)
    uc_idx = uc_idx_vec(uc_idx_m);
    switch uc_idx
        case 0
            FolderName = 'Captures/07302021_1Mbps/1m_3'; true_distance = 1; Fs = 4e6;
        case 1
            FolderName = 'Captures/07302021_1Mbps/5m_1'; true_distance = 5; Fs = 4e6;
        case 2
            FolderName = 'Captures/08042021_1Mbps/3m/24_22'; true_distance = 3; Fs = 4e6;
        case 3
            FolderName = 'Captures/08042021_1Mbps/3m/24_24'; true_distance = 3; Fs = 4e6;
        case 4
            FolderName = 'Captures/08042021_1Mbps/3m/24_25'; true_distance = 3; Fs = 4e6;
        case 5
            FolderName = 'Captures/08042021_1Mbps/3m/24_26'; true_distance = 3; Fs = 4e6;
        case 6
            FolderName = 'Captures/08042021_1Mbps/3m/24_27'; true_distance = 3; Fs = 4e6;
        case 7
            FolderName = 'Captures/08042021_1Mbps/9m/18_46'; true_distance = 9; Fs = 4e6;
        case 8
            FolderName = 'Captures/08042021_1Mbps/9m/18_47'; true_distance = 9; Fs = 4e6;
        case 9
            FolderName = 'Captures/08042021_1Mbps/9m/18_49_warning'; true_distance = 9; Fs = 4e6;
        case 10
            FolderName = 'Captures/08042021_1Mbps/9m/18_50'; true_distance = 9; Fs = 4e6;
        case 11
            FolderName = 'Captures/08042021_1Mbps/9m/18_52'; true_distance = 9; Fs = 4e6;
        otherwise
            error('Use case not existent');
    end
    
    if Fs == 4e6
        Latency = 2*40*4/Fs + 2*3.6e-6 + 2*12*4/Fs + 1.859e-6;
    else
        Latency = 0; % To be determined for 2Mbps captures
    end

    disp('------------------------')
    disp(['Reading ', FolderName, 'true dist = ', num2str(true_distance), 'm'])

    a = dir(FolderName);
    for m = 1:length(a)
        if contains(a(m).name,'initiator')
            FileNameInit = a(m).name;
        end
        if contains(a(m).name,'reflector')
            FileNameRefl = a(m).name;
        end
    end
    [ResI,RssiI,AgcIdxI] = RttReadLog(fullfile(FolderName,FileNameInit));
    [ResR,RssiR,AgcIdxR] = RttReadLog(fullfile(FolderName,FileNameRefl));
    Rssi0VecI(uc_idx_m) = RssiI(1);
    Rssi1VecI(uc_idx_m) = RssiI(2);
    AgcIdxVecI(uc_idx_m) = AgcIdxI;
    Rssi0VecR(uc_idx_m) = RssiR(1);
    Rssi1VecR(uc_idx_m) = RssiR(2);
    AgcIdxVecR(uc_idx_m) = AgcIdxR;
    
    RttResI = RttUnpackRes(ResI);
    RttResR = RttUnpackRes(ResR);
%     figure(101); plot(s2v(RttResI,'rtt_cfo')*Fs/4/2); hold on;
%     figure(102); plot(s2v(RttResR,'rtt_cfo')*Fs/4/2); hold on;
    if Fs == 4e6 % 1Mbps
        load BLERTT_1M_Data_Simulation_03112021_TF_cbpf_IF1p0_BW698_T25;
    else % 2Mbps
        load BLERTT_2M_Data_Simulation_03112021_TF_cbpf_IF1p5_BW1236_T25;
    end
    RttTimeInt = zeros(size(RttResI));
    RttTimeFrac = zeros(size(RttResI));
    RttTime = zeros(size(RttResI));
    RttTimeI = zeros(size(RttResI));
    RttCfoI = zeros(size(RttResI));
    RttRssiI = zeros(size(RttResI));
    RttAcgIdxI = zeros(size(RttResI));
    RttTimeR = zeros(size(RttResI));
    RttCfoR = zeros(size(RttResI));
    RttRssiR = zeros(size(RttResI));
    RttAcgIdxR = zeros(size(RttResI));
    for m = 1:length(RttResI)
        % rtt_time_est is the fractional delay estimation implementation
        RttResI(m).rtt_frac_dly = rtt_time_est(RttResI(m).rtt_p_delta,de2bi(hex2dec(RttResI(m).pn),32),Gamma);
        RttResR(m).rtt_frac_dly = rtt_time_est(RttResR(m).rtt_p_delta,de2bi(hex2dec(RttResR(m).pn),32),Gamma);
        % Convert integer time stamps to microseconds
        ToD_I = RttResI(m).T1/32e6;
        ToA_I = RttResI(m).T2/32e6;
        ToD_R = RttResR(m).T2/32e6;
        ToA_R = RttResR(m).T1/32e6;
        % Combine fractional delay and integer adjustment from the HARTT block and convert to seconds
        ToA_I_frac = (RttResI(m).rtt_frac_dly + RttResI(m).rtt_int_adj)/Fs;
        ToA_R_frac = RttResR(m).rtt_frac_dly/Fs + RttResR(m).rtt_int_adj/Fs;
        % Compute ToF (fractional time-stamp is added to the integer time-stamp)
        RttTime(m) = (ToA_I+ToA_I_frac) - ToD_I + ...   % Initiator ToA-ToD
                     (ToA_R+ToA_R_frac) - ToD_R;        % Reflector ToA-ToD
                      RttTime(m) = mod(RttTime(m),2^16/(32e6))  - Latency;  % Remove TPM2 counter ambiguity
        % Debug (ToA-ToD) time-stamps computation 
        RttTimeInt(m) = ToA_I - ToD_I + ToA_R - ToD_R;  RttTimeInt(m) = mod(RttTimeInt(m),2^16/(32e6))  - Latency;
        RttTimeI(m) = (ToA_I+ToA_I_frac) - ToD_I; RttTimeI(m) = mod(RttTimeI(m),2^16/(32e6));
        RttTimeR(m) = (ToA_R+ToA_R_frac) - ToD_R; RttTimeR(m) = mod(RttTimeR(m),2^16/(32e6));
        RttCfoI(m) = RttResI(m).rtt_cfo*Fs/4/2; %RttResI(m).leg_cfo*Fs/4;
        RttCfoR(m) = RttResR(m).rtt_cfo*Fs/4/2; %RttResR(m).leg_cfo*Fs/4;
        
        disp(['step:', num2str(ResI(m).stepno, '%02d'), ...
              ' fracI=' num2str(round(((RttResI(m).rtt_frac_dly + RttResI(m).rtt_int_adj)/Fs)*1e9)), 'ns', ...
              ' fracR=',num2str(round(((RttResR(m).rtt_frac_dly + RttResR(m).rtt_int_adj)/Fs)*1e9)), 'ns', ...
              ' ToF=', num2str(round(RttTime(m)*1e9)), 'ns', ...
              ' dist=', num2str((RttTime(m)/2)*3e8, '%03.2f'), 'm'])
    end
    
    if length(uc_idx_vec) > 1
        % Dbg
        ToF = RttTime/2;
        figure(true_distance);
        if uc_idx_m == 1 
            plot(true_distance*ones(size(ToF)),'k--','linewidth',2); hold on;
            true_distance_reg = true_distance;
        else
            if true_distance ~= true_distance_reg
                plot(true_distance*ones(size(ToF)),'k--','linewidth',2); hold on;
                true_distance_reg = true_distance;              
            end
        end
        
        plot(ToF*1e9/3.3,'x');
        grid on; xlabel('Step #'); title(['ToF distance estimate (m) - true distance = ' num2str(true_distance) ' meters']);
        y = ToF*1e9/3.3 - true_distance;
        legend('True distance','Capture #1','Capture #2','Capture #3','Capture #4','Capture  #5');
        x = -4:0.25:4;
        H = cumsum(hist(y,x)); H = H/max(H);
        figure(420); 
        if true_distance == 3
            hx(1) = subplot(121);
            plot(x,H); grid on; hold on; title('Distance error CDF - 3m');
            legend('#1','#2','#3','#4','#5');
        else
            hx(2) = subplot(122);
            plot(x,H); grid on; hold on; title('Distance error CDF - 9m');
            legend('#1','#2','#3','#4','#5');
        end
        
        figure(421);
        ha(1)=subplot(221); plot((RttTimeI-mean(RttTimeI))*1e9/3.3); hold on; grid on; title('Init. dist. err. (m)'); ylim([-6,6]);
        ha(2)=subplot(222); plot((RttTimeR-mean(RttTimeR))*1e9/3.3); hold on; grid on; title('Refl. dist. err. (m)'); ylim([-6,6]);
        ha(3)=subplot(223); plot(RttCfoI); hold on; grid on; title('Init. CFO (Hz)');  ylim([-3000,3000]);
        ha(4)=subplot(224); plot(RttCfoR); hold on; grid on; title('Refl. CFO (Hz)');  ylim([-3000,3000]);
        linkaxes(ha,'x');
    else
        
        figure('Name',['Time of flight - ' FolderName]);
        ToF = RttTime/2;
        ToFInt = RttTimeInt/2;
        plot(ToF*1e9/3.3,'x-'); hold on;
        %plot((ToFInt)*1e9/3.3,'o-');
        plot(true_distance*ones(size(ToF)),'k--','linewidth',2);
        legend({['ToF distance - Avg=' num2str(mean(ToF*1e9/3.3))], ...
            %['ToF coarse distance - Avg=' num2str(mean(ToFInt*1e9/3.3))], ...
            ['True distance = ' num2str(true_distance)]});
        grid on;
        xlabel('Packet #'); title('ToF (m)');  title(['Distance (meters) (' FolderName ')'],'interpreter','none');
    end
end
figure(422);
subplot(211); plot(Rssi0VecI,'bs-'); hold on; plot(Rssi1VecI,'bx--');
              plot(Rssi0VecR,'ro-'); plot(Rssi1VecR,'r*--'); grid on;
              title('RSSI'); legend('Init. pkt0','Init. pkt1','Refl. pkt0','Refl. pkt1');
subplot(212); plot(AgcIdxVecI,'bs-'); hold on; plot(AgcIdxVecR,'ro-'); grid on;
              title('AGC index'); legend('Init.','Refl.');


% return;  
if length(uc_idx_vec) == 1
    figure('Name',['Integer vs fractional - ' FolderName]);
    ha(1)=subplot(321); plot(mod(s2v(RttResI,'T2')-s2v(RttResI,'T1'),2^16),'bs-'); title('Initiator ToA - ToD (32MHz cycles)'); grid on;
    ha(2)=subplot(322); plot(mod(s2v(RttResR,'T2')-s2v(RttResR,'T1'),2^16),'bs-'); title('Reflector ToD - ToA (32MHz cycles)'); grid on;
    ha(3)=subplot(323); plot((s2v(RttResI,'rtt_int_adj')+s2v(RttResI,'rtt_frac_dly'))*32e6/Fs,'r-'); title('Initiator fractional delay (32MHz cycles)'); grid on;
    ha(4)=subplot(324); plot((s2v(RttResR,'rtt_int_adj')+s2v(RttResR,'rtt_frac_dly'))*32e6/Fs,'r-'); title('Reflector fractional delay (32MHz cycles)'); grid on;
    ha(5)=subplot(325); plot((RttTimeI - mean(RttTimeI))*1e9/3.3); grid on; title('Initiator combined (meters)');
    ha(6)=subplot(326); plot((RttTimeR - mean(RttTimeR))*1e9/3.3); grid on; title('Reflector combined (meters)');
    linkaxes(ha,'x');
    
    figure('Name',['HARTT vs legacy - ' FolderName]);
    subplot(211); plot((s2v(RttResI,'rtt_frac_dly') + s2v(RttResI,'rtt_int_adj'))*32e6/Fs,'linewidth',2); hold on; plot(-(s2v(RttResI,'leg_frac'))*32e6/Fs,'linewidth',2);
    title('Initiator fractional delay estimate (32MHz cycles)'); legend('HARTT','Legacy'); grid on;
    subplot(212); plot((s2v(RttResR,'rtt_frac_dly') + s2v(RttResR,'rtt_int_adj'))*32e6/Fs,'linewidth',2); hold on; plot(-(s2v(RttResR,'leg_frac'))*32e6/Fs,'linewidth',2);
    title('Reflector fractional delay estimate (32MHz cycles)'); legend('HARTT','Legacy'); grid on;
    
    figure('Name',['Differential integer timestamps - ' FolderName]);
    TxDiff = diff(s2v(RttResI,'T1'));
    RxDiff = diff(s2v(RttResI,'T2'));
    TxDiff(TxDiff < -2^15) = TxDiff(TxDiff < -2^15) + 2^16;
    RxDiff(RxDiff < -2^15) = RxDiff(RxDiff < -2^15) + 2^16;
    subplot(211); plot(TxDiff); hold on; plot(RxDiff); title('Initiator differential time-stamps (32MHz cycles)'); legend('\DeltaToD','\DeltaToA'); grid on;
    TxDiff = diff(s2v(RttResR,'T2'));
    RxDiff = diff(s2v(RttResR,'T1'));
    TxDiff(TxDiff < -2^15) = TxDiff(TxDiff < -2^15) + 2^16;
    RxDiff(RxDiff < -2^15) = RxDiff(RxDiff < -2^15) + 2^16;
    subplot(212); plot(TxDiff); hold on; plot(RxDiff); title('Reflector differential time-stamps (32MHz cycles)'); legend('\DeltaToD','\DeltaToA'); grid on;
    
    figure('Name',['ToD vs ToA - ' FolderName]);
    ha(1)=subplot(221); plot(mod(s2v(RttResI,'T2')-s2v(RttResI,'T1'),2^16),'bs-'); title('Initiator ToA - ToD (32MHz cycles)'); grid on;
    ha(2)=subplot(223); plot([0 diff(s2v(RttResI,'T1'))]); hold on; plot([0 diff(s2v(RttResI,'T2'))]); legend('Init T1 (ToD)','Init T4 (ToA)'); grid on;
    ha(3)=subplot(222); plot(mod(s2v(RttResR,'T2')-s2v(RttResR,'T1'),2^16),'bs-'); title('Reflector ToD - ToA (32MHz cycles)'); grid on;
    ha(4)=subplot(224); plot([0 diff(s2v(RttResR,'T1'))]); hold on; plot([0 diff(s2v(RttResR,'T2'))]); legend('Refl T2 (ToA)','Refl T3 (ToD)'); grid on;
    linkaxes(ha,'x');
end
    
