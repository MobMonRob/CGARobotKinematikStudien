// Clifford Algebra with 4,1 metric for 3D CGA. 
Algebra(4,1,()=>{

	// definition of a null basis and upcasting for points and spheres
	var ni = 1e4+1e5, no = .5e5-.5e4
	var up = (x)=>no+x+.5*x*x*ni
	var sphere = (P,r)=>!(P-r**2*.5*ni)

	// definition of pe and p5
	var pe = up(1e1+1e2+1e3)
	var p5 = up(2e2+1e3)

	// definition of the spheres sc and k0
	var sc = sphere(p5, 1.0915) // conversion of d4 from mm in decimeter to fit on the screen
	var k0 = !(no - ((!sc << no)*ni))

	// intersection of sc and k0 to find the circle c5k
	var c5k = sc & k0

	// intersection of the circle e5k with the horizontal plane through p5
	var qc = (c5k & (p5 ^ 1e1 ^ 1e2 ^ ni))

	// selection of pc
	var pc = (qc + Math.sqrt(qc**2)) / (-ni << qc)

	// definition of the vertical plane pic
	var pic = no ^ 1e3 ^ pc ^ ni
	
	// Graph the items
	document.body.appendChild(this.graph([
		0xFF0000, sc, "sc", k0, "k0",
		0x008800, c5k, "c5k",
		0x0000FF, c5k, "c5k",
		0x0000FF, qc, "qc",
		0xFCBA03, pc, "pc", pic, "pic",
	],{conformal:true,gl:true,grid:true})); 
});
