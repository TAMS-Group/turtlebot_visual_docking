// Written by Kolja Poreski
//<9poreski@informatik.uni-hamburg.de>

 difference(){
        translate([0,0,0]){
            linear_extrude(height = 8)
            resize([0,207,0],auto=true)
            import("Halterung.dxf");
        }
        union(){
            translate([0,7,0]){
            cube([400,2*57,400],center=true);
            }
            translate([0,206-22+5,0]){
                cube([400,44,400],center=true);
            }
            for(i=[-1,1]) {
            translate([19, 106+i*15, -0.2]) {
                cylinder(r=2.3, h=1000, center=true, $fn=50);
                cylinder(r=5.0, h=4.0, center=false, $fn=50);
            }
        }   
       }
}