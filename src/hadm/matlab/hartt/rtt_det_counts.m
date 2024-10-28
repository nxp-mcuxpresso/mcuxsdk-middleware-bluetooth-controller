function C = rtt_det_counts(aa_bin)
% Extended AA (adding the last bit of the preamble and the first bit of the PDU
% Assumes that the first bit of the PDU is the last bit of the AA inverted
aa_ext = [~aa_bin(1) aa_bin ~aa_bin(length(aa_bin))];
% Init counts with zero
cnt_010 = 0; cnt_011 = 0; cnt_111 = 0;
% Parse sequence
for m = 2:length(aa_ext)-1
    bit_0 = aa_ext(m);
    bit_m1 = aa_ext(m-1);
    bit_p1 = aa_ext(m+1);
    if (bit_0 ~= bit_m1) && (bit_0 ~= bit_p1)
        cnt_010 = cnt_010 + 1;
    elseif (bit_0 == bit_m1) && (bit_0 == bit_p1)
        cnt_111 = cnt_111 + 1;
    else % ((bit_0 ~= bit_m1) && (bit_0 == bit_p1)) || ((bit_0 == bit_m1) && (bit_0 ~= bit_p1))
        cnt_011 = cnt_011 + 1;
    end
end
% Assign counts to vector
C = [cnt_010,cnt_011,cnt_111];