function [ch_dly_est,coeff] = rtt_time_est(p_delta,pn_seq,Gamma,precision)
%% ------------------------------------------------------------------------
% Description: fractional delay estimation function
%
% Inputs:
%   p_delta (Nx1) - vector containing the p_delta values
%   pn_seq (Nx32) - matrix containing the 32-bit sequences along the rows
%   Gamma (6x4) - the transformation matrix from sequence properties to coefficients
%   precision - 0 --> single precision; 1 --> double precision
%
% Outputs:
%   ch_dly_est (Nx1) - vector containing the fractional delay estimates
%   coeff (Nx4) - matrix containing the fit coefficients
%
% Dependencies of other functions:
%   RttDetCounts
%
% REVISIONS:
%   v0.0 - Mihai Stanciu (nxa17611) - 12/08/2020 - Initial version 
%   v0.1 - Mihai Stanciu (nxa17611) - 01/11/2021 - Added single precision option 
%% ------------------------------------------------------------------------
if nargin < 4
    precision = 1;
end
ch_dly_est = zeros(size(p_delta));
coeff = zeros(length(p_delta),4);
for m = 1:length(p_delta)
    cnt = rtt_det_counts(pn_seq(m,:));
    if precision == 0
        coeff_fit = single(single([cnt cnt.^2])*single(Gamma));
    else
        coeff_fit = ([cnt cnt.^2])*Gamma;
    end
    coeff(m,:) = coeff_fit.';
    p_dlt = p_delta(m);
    if precision == 0
        ch_dly_est(m) = single((p_dlt.^(0:3))*(coeff_fit.'));
    else
        ch_dly_est(m) = (p_dlt.^(0:3))*(coeff_fit.');
    end
end