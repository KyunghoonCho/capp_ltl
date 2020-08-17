% Update heuristic weight.
function w = updateHeuristicWeight(h_z,area_g,cov,nrsel,alpha,beta,gamma)
numerator           = area_g;
denominator_term1   = (1+cov.^2).^0;
denominator_term2   = (1+nrsel);
denominator_term3   = (1+h_z).^0;

denominator_term1   = denominator_term1.^alpha;
denominator_term2   = denominator_term2.^beta;
denominator_term3   = denominator_term3.^gamma;
w = numerator./(denominator_term1.*denominator_term2.*denominator_term3);
end
