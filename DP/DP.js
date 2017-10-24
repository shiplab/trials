/*   ////////
    Class definition
 
 */



 /**
  * 
  * 
  * @class Control
  * class, it will calculated a control signal, based on linear control theory, so a desired variable goes to the a specific value(setpoint)
  * @see Handbook of Marine Craft Hydrodynamics and Motion Control,Fossen,2014, pg.396-416 or Modern Control Theory, Brogan, 1974 for Designing the controller with this class
  */
 class Control{
     //////////////////////////////////////////////////////////////////////////////////////////
     //Controller class, it will calculated a control signal, based linear control theory, so a desired variable goes to the a specific value(setpoint)
   
    ///////
    //Input : setpoint, variable to control, Control poles,zeros and Gain.
    //Output: control signal
    /////////////////////////////////////////////////////////////////////////////////////////

          /**
           * Creates an instance of Control.It will calculated a control signal, based on  linear control theory, so a desired variable goes to the a specific value(setpoint)
           * @see Handbook of Marine Craft Hydrodynamics and Motion Control,Fossen,2014, pg.396-416 or Modern Control Theory, Brogan, 1974 for Designing the controller with this class
           * @param {any} k -proporcional gain
           * @param {any} setpoint -setpoint 
           * @memberof Control
           */
          constructor(k,setpoint){
              this.u = [];                                        //control signal memory
              this.ku=[];                                        //control signal pol definition
              this.e = [];                                      //
              this.ke=[];
              this.K = k;
              this.setpoint = setpoint;
          }

          AddFilter(poles,zeros,gain,place){
              if(place == "output"){
                 this.H ={"poles":poles,"zeros":zeros,"gain":1};
              }else{
                 this.F={"poles":poles,"zeros":zeros,"gain":1};
              }
            
          }
     
          /**
           * Reset the memory of the controller
           *
           * @memberof Control
           */
          Reset(){
              this.u =[];
              this.e = [];
              for(var i =0;i < this.ku.length;i++){
                  this.u.push(0);
                  this.e.push(0);
              }

          }

          /**
           * 
           * Changes the setpoint of the controller
           * @param {any} setpoint 
           * @memberof Control
           */
          changeRef(setpoint){
                    this.setpoint=setpoint;

                }
          /**
           * 
           * Configurate the memory and gains for the input of the control law
           * @param {Array} ku - gains of the input memory as defined in [Brogan,1974] 
           * @memberof Control
           */
          setPoles(u){
              for(var i=0;i< u.length;i++){
                  this.ku.push(u[i]);
                  this.u.push(0);
              }

          }
          /**
           * 
           * Configurate the memory and gains for the error of the control law.
           * @param {Array} ke - gains of the error memory as defined in [Brogan,1974]
           * @memberof Control
           */
          setZeros(e){
              
             for(var i=0;i< e.length;i++){
                  this.ke.push(e[i]);
                  this.e.push(0);
              } 
          }

          /**
           * 
           * Calculation of the control action for one timestep
           * @param {any} y -control variable current output.
           * @returns control action calculated by the control law for current timestep
           * @memberof Control
           */
          ControlAction(y){
              
              
             this.e.unshift(this.setpoint-y);
             var sumu=0,sume=0;
             for(var i=0;i< this.u.length;i++){
                 if(i != 0){
                 sumu += (-this.ku[i]*this.u[i]);
                 }
                 sume += this.ke[i]*this.e[i]; 
             }
             
             this.u.unshift((this.K*sume+sumu)/this.ku[0]);
             this.u.pop();
             this.e.pop();
             
             return this.u[0];


          }
      }  
      class Thruster {
    //////////////////////////////////////////////////////////////////////////////////////////
     //thruster class, it will calculated the dynamic of the thruster for a specific rpm, also given a specific force, it will say the necessary rpm for this thruster to exerce that force
        
    ///////
    //Input : Ct(efficiency, between 0 and 1), Diameter of the propeller,maximum allowed power, max allowed rotation, water density, rotation, force.
    //Output: Force, rotation.
    /////////////////////////////////////////////////////////////////////////////////////////

                constructor(Ct,Diameter,maxkw, maxrpm){
                    this.Ct = Ct;                                       // eficiency coefficient between 0 and 1
                    this.Diameter = Diameter;                           // Diameter of the propeler in meter
                    this.maxkw = maxkw;                                 // maximum allowed power in kilo Watts
                    this.maxrpm = maxrpm;                               // maximum allowed rotation of the propeller in rotation per minute.
                    
                }
                
                

                setD(Diameter){
                    this.Diameter = Diameter;
                }

                setrpm(maxrpm){
                    this.maxrpm = maxrpm;
                }

                setkw(maxkw){
                    this.maxkw = maxkw;
                }

                 kw2rpm(inputkw){
                     //Linear 
                    if(inputkw >= this.maxkw){
                        return this.maxkw;
                    }else{
                        return (inputkw*this.maxrpm)/(this.maxkw);
                    }

                }
               rpm2kw(inputrpm){
                    if(inputrpm > this.maxrpm){
                        return this.maxrpm;
                    }else{
                        return (inputrpm*this.maxkw)/(this.maxkw);
                    }   
                }
                rpmsat(inputrpm){
                    if(Math.abs(inputrpm) > this.maxrpm){
                        return this.maxrpm;
                    }else return inputrpm;
                }
                CalTrust(inputrpm,rho){

                    if(inputrpm < 0){
                    return -(Math.pow(this.rpmsat(inputrpm),2)*Math.pow(this.Diameter,4)*(rho)*this.Ct);
                    }
                    return (Math.pow(this.rpmsat(inputrpm),2)*Math.pow(this.Diameter,4)*(rho)*this.Ct);


                }

                Trust2rpm(trust,rho){
                    if(trust < 0){
                         return -(Math.sqrt(-trust/(rho*this.Ct))/(Math.pow(this.Diameter,2)));

                    }else
                        return (Math.sqrt(trust/(rho*this.Ct))/(Math.pow(this.Diameter,2)));

                }
                    

        }

        /*   ////////
    Parameters definition
 
 */

    /**
     * 
     * 
     * @class 
     * Creates the model and the controllers of the dynamic positioning(DP) systems of a simplified vessel system. 
     * @see pg.152 of Handbook Marine Craft Hydrodynamics and Motion Control,Fossen,2014 for futher information. 
     * @version 0.1 (alfa)
     */
    class DP{

        /**
         * 
         * Creates an instance of DP system. uses 3 {@link Control} and 3 {@link Propeller} to implement the dynamic postioning of the ship.
         * @param {Ship} Ship - The ship Object
         * @param {any} setpointx -setpoint for the position in the x axis
         * @param {any} setpointy -setpoint for the position in the y axis
         * @param {any} setpointyaw -setpoint for the position in the z axis
         * @memberof DP
         */
        constructor(Ship,setpointx,setpointy,setpointyaw){
                this.t =0;                                                       // time simulated ;
                this.output = {};
                this.outputArray =[];
                this.thruster1 = {};
                this.thruster2 = {};
                this. thruster3 = {};                                                     
                
                    this.thruster1 = new Thruster(1,1.05,330,2100),                  // thruster dynamics, need the capabilities like eficiency, diameter of the propeller, maximum power allowed, maximum rotation allowed, 
                    this.thruster2 = new Thruster(1,1.05,330,2100),
                    this.thruster3 = new Thruster(1,1.05,330,2100);
              
                this.controlx = new Control(7.5407,setpointx),                         // Controllers of Sway
                this.controlyaw = new Control(7.5407,setpointyaw),                        // Controller of Yaw
                this.controly = new Control(7.5407,setpointy);                         // Controller of Surge

                if(Ship.Methods.Holltrop == undefined){
                    this.rho = 1000,                                                     // water density
                    this.Cd = Ship.VesselData.BlockCoefficient,
                    this.Ax = Ship.VesselData.BOA*Ship.VesselData.T;                                                             //Crosssection Area of the Ship in water with x as normal vector
                    this.Ay = Ship.VesselData.LOA*Ship.VesselData.T;                                                             //Crosssection Area of the Ship in water with y as normal vector
                }else{
                    this.rho = 1000,                                                     // water density
                    this.Cd = Ship.VesselData.BlockCoefficient,
                    this.Ax = Ship.VesselData.BOA*Ship.VesselData.T;                                                             //Crosssection Area of the Ship in water with x as normal vector
                    this.Ay = Ship.VesselData.LOA*Ship.VesselData.T;    

            }
                this.m = parseFloat(document.getElementById('WeightLight').innerHTML);
                this.I = Math.pow((0.28*Ship.VesselData.LOA),2)*this.m;
                this.deltat=0.001;
                this.output = {"t": 0, "px": 0, "py": 0, "vx": 0, "vy": 0,"yaw": 0, "vyaw": 0};
                this.controlx.setZeros([1,-1]);
                this.controlx.setPoles([1,-0.7967]);

                this.controlyaw.setZeros([1,-1]);
                this.controlyaw.setPoles([1,-0.7967]);
                
                this.controly.setZeros([1,-1]);
                this.controly.setPoles([1,-0.7967]);
        }
               
   
        /**
         * 
         * function contain the model of the DP system. Uses feedback linearization method with linear controller to Dynamic Positioning.
         * @see {@link https://cse.sc.edu/~gatzke/cache/npc-Chapter4-scan.pdf} for futher information on feedback linearization 
         * @memberof DP
         */
        model(){
            ////////////////////////////////////////////////////////////////
            //// diference equations that define  the dynamic of the Ship  due to force of the propellers
            //Input:
            //Output: position (x,y,yaw), speed(vx,vy,yaw speed)
            //////////////////             
          
                
            var accx;                                           // acceleration in x axis
            var xdrag= (this.rho*Math.pow(this.output.vx,2)*this.Cd*this.Ax)/2;    // need to be changed by a Holltrop method, Resistence force
            var nx=0;                                           //Propeller force in x

            var accy;                                           // acceleration in y axis
            var ydrag = (this.rho*Math.pow(this.output.vy,2)*this.Cd*this.Ay)/2;   // need to be changed by Holltrop method, Resistence force
            var ny =0;

            var accw;                                           //angular acceleration in z axis (yaw)
            var wdrag = 0;                                      // need to be changed by holltrop method, resistence torque
            var nyaw = 0;                                       // propeller torque in yaw
                
            if(this.output.vx > 0){
                 nx  = this.thruster1.Trust2rpm((this.m*this.controlx.ControlAction(this.output.px)+xdrag),this.rho); // linearize the thruster behaviour, compensate the drag( Non-linear Observer)
                     
                 accx = (this.thruster1.CalTrust(nx,this.rho)-xdrag)/this.m;
                 
            }else{
                 nx  = this.thruster1.Trust2rpm(this.m*this.controlx.ControlAction(this.output.px)-xdrag,this.rho);
                 accx = (this.thruster1.CalTrust(nx,this.rho)+xdrag)/this.m;
            }

           
            if(this.output.vy > 0){
               
              ny  = this.thruster2.Trust2rpm((this.m*this.controly.ControlAction(this.output.py)+ydrag),this.rho);
             
             accy = (this.thruster2.CalTrust(ny,this.rho)-ydrag)/this.m;
            }else{
               
                ny  = this.thruster1.Trust2rpm((this.m*this.controly.ControlAction(this.output.py)-ydrag),this.rho);
               
                accy = (ydrag+this.thruster2.CalTrust(ny,this.rho))/this.m;
               
                
            }

           
             
            nyaw = this.thruster3.Trust2rpm((this.I*this.controlyaw.ControlAction(this.output.yaw)),this.rho);
            accw = this.thruster3.CalTrust(nyaw,this.rho)/this.I;
           

            
                
                this.output.vx += (accx*this.deltat);
                this.output.px += (this.output.vx*this.deltat);

                this.output.vy += (accy*this.deltat);
                this.output.py += (this.output.vy*this.deltat);

                this.output.vyaw += (accw*this.deltat);
                this.output.yaw += (this.output.vyaw*this.deltat);                
                this.output.t += this.deltat;
                
             
            
           
            
        }        
          
          
           
            
       
       

        /**
         * 
         * 
         * @returns the position,angle,speed and simulation time
         * @memberof DP
         */
        CurrentPos(){
            //return the position,angle,speed and time
            return this.output;


        }
        /**
         * 
         * Simulate the DP until specified time
         * @param {any} time
         * @returns the position,angle,speed of each timestep until the specified time
         * @memberof DP
         */
        UpdatePosT(time){
            // update the postion and the angle of the vessel based on the model of the system until the simulated time, return the time, position,angle and speeds(linear and angular)
                var N = Math.floor((time-t)/this.deltat);
                if(N < 0 ){return this.output;}
                else{

                      for(i = 0; i < N; i++){
                          this.model();
                      }
                      this.outputArray.push(this.CurrentPos());
                      return this.output;  
                }



        }

        /**
         * 
         * Simulate the DP system for N timesteps
         * @param {any} N -number of time steps to simulate
         * @returns the position,angle,speed and time of the simulation
         * @memberof DP
         */
        UpdatePosN(N){
             // update the postion and the angle of the vessel based on the model of the system for N interations, return the time, position,angle and speeds(linear and angular)
                for(var i = 0; i < N; i++){
                          this.model();
                      }
                this.outputArray.push(JSON.parse(JSON.stringify(this.output)));
                return this.output;  
        }
        




    }
        
   
        function DPup(){
                var last = ship.Methods.DP.UpdatePosN(1000);
                var vessels = ship.Methods.DP.outputArray;
             var margin = {top: 20, right: 20, bottom: 30, left: 40},
                width = 960 - margin.left - margin.right,
                height = 500 - margin.top - margin.bottom;
               d3.select("body").select("svg").remove();

            var x = d3.scaleLinear().range([0,width]);
            var y = d3.scaleLinear().range([height,0]);
        
                     var svg = d3.select("body").append("svg")
                        .attr("width", width + margin.left + margin.right)
                        .attr("height", height + margin.top + margin.bottom)
                        .append("g")
                        .attr("transform", "translate(" + margin.left + "," + margin.top + ")")       

           

                      

           
          

            x.domain([d3.min(vessels,function(d){return d.px;})-1, d3.max(vessels, function(d){return d.px;})+1]);
            y.domain([d3.min(vessels, function(d){return d.py;})-1, d3.max(vessels, function(d){return d.py;})+1]);

            svg.selectAll("dot").data(vessels).enter().append("circle")
                .attr("r", 2)
                .attr("cx", function(d) { return x(d.px); })
                .attr("cy", function(d) { return y(d.py); });

            

            // x-axis
            svg.append("g")
                .attr("transform", "translate(0," + height + ")")
                .call(d3.axisBottom(x));

            // Add the Y Axis
            svg.append("g")
                .call(d3.axisLeft(y));


 var GM = svg.append("rect").attr("x",(x(last.px)-5))
                           .attr("y",(y(last.py)-20))
                           .attr("transform", "rotate("+ last.yaw*180/Math.PI +"," +(x(last.px)+5)+ ","+(y(last.py)+20)+")")
                           .attr("width",10)
                           .attr("height",40)
                           .attr("fill","blue");
         
        

        }

        function DPstart(){
            

                    var x = document.getElementById("dpxsetpoint").value;
                    var y = document.getElementById("dpysetpoint").value;
                     
                    ship.Methods.DP = new DP(ship,x,y,0);
                   
                   
                    var dpflag = setInterval(function(){DPup()},1000);
                    //DPup();
                    

        }

      