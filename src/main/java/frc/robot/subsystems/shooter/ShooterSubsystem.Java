// 6 motores en total

// 3 para flywheels en coast mode

// 1 para hood (la que ajusta en angulo de tiro)
// Este usa un ecoder absoluto como referencia.

// 2 para indexer. No conviene separarlos por mecanismo 
// por que el feeder del shooter y el qe tenemos en el piso 
// se activan al mismo tiempo y tendriamos un subystema extra 
// sin mucho sentido.

// Todos son motores vortex brushless

// devemos tener un metodo que permita ajustar el angulo del hood.

// otro que permita setear cada shooter a una rpm. Cuando alcance 
// las rpm o este muy cerca se debe cambiar el status de una variable 
// booleana a readyToShoot.

// final ajustar disparo recibiendo distancia.
// feedear

//feeder config


//move feeder
// moter.set()