function Lo = DrawTendons(Pa,Pb,col,L)
% Pa/b = [x,y,z; x,y,z...]
% p = [pl; pr; tl; tr];

%ordering of matrix:
% pan Left (pl), pan right (pr);
% tilt Left (tl), tilt right (tr);

%Turn matrix into Length vector
dL = reshape(L',[size(Pa,1),1]);
%pl; pr; tl; tr;

for ii = 1:size(Pa,1)
    %Get Pa(ii,:) and Pb(ii,:)
    A = Pa(ii,:); B = Pb(ii,:);
    %Plot line between them using colour provided they are not null points
    if ~(all(A==0)&&all(B==0))
        hold on
        plot3([A(1) B(1)],[A(2) B(2)],[A(3) B(3)],col(ii,:))
        hold on
        %Calculate Magnitude:
        dx = B(1)-A(1); dy = B(2)-A(2); dz = B(3)-A(3);
        %Add the length to the totals
        dL(ii) = dL(ii) + norm([dx dy dz]);        
    end
end

%Return shape of the matrix back to normal:
Lo = reshape(dL,size(L))';
end