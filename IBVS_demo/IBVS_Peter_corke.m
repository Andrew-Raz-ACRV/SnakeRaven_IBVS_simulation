%Peter Corke Textbook visual servoing
%Create camera at pose Tc0
cam = CentralCamera('default');
Tc0 = transl(1,1,-3)*trotz(0.6);
cam = cam.move(Tc0);
%Create target
P = mkgrid( 2, 0.5, 'T', transl(0,0,3) );
%Create desired image
pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');

%IBVS function
ibvs = IBVS(cam,'pstar', pStar);
ibvs.run();
view(3)
figure
ibvs.plot_p();