
function Fuel_Consumption(SFC,MCR,Speed,Dist){
    // SFC fuel specific comsuption
      
    var len = Dist.length;
    
    var sum =0;
    var com = 0;
    var cons =[];
    for(var i =0; i < len;i++){
        var time;
        if(Speed.length> 1 && Speed.length !== 'undefined'){
            time =  Dist[i]/Speed[i];
            
        }else{
            time = Dist[i]/Speed;
        }
        cons[i] = time*MCR*SFC; // partial consumption
        com += cons[i]; // total consumption 
    }
    var result = {"Total": com, "Consumption": cons};
    return result;

}

function Path_FC(SFC,MCR,Speed,Path){
    if(Path.length <= 1 || Path.length == 'undefined'){
        return -1;

    }else{
        var Dist = [];
        for(var i = 1; i < Path.length; i++){
            var aux = Math.pow(Path[i].x - Path[i-1].x,2)+Math.pow(Path[i].y - Path[i-1].y,2);
            Dist.push(Math.sqrt(aux));

        
        }
        console.log('------------------------------------');
        console.log(Dist);
        console.log('------------------------------------');
        return Fuel_Consumption(SFC,MCR,Speed,Dist);
    }

}

function Path_Build(x,y){
    var result =[];
    for(var i =0; i < x.length;i++){
       result[i] = {"x": x[i],"y": y[i]};
    }
    return result;
}
