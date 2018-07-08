require(tidyverse)
require(magrittr)


calculateSmoothing = function(updateInterval, halfTime) {
  # smooth
  # updateInterval [=] ms; delta time between update to this function
  # halfTime [=] ms; delta time for smoothed signal to transition halfway to new value
  samples = halfTime / updateInterval / log(2.0)
  alpha = 1.0-(samples-1.0)/samples
  return( alpha )
}
performSmoothing = function(avg, x, alpha) {
  dif = x - avg
  incr = alpha * dif
  avg = avg + incr
  return( avg )
}

updateInterval = 20 # ms
halfTime = 500 # ms
alpha = calculateSmoothing(updateInterval, halfTime)

sim = tibble(
	i=0:100,
	x=i*halfTime
) %T>% print

ramp = function(y0, 


samples = 1000 / 20 / log(2)
alpha = 1-(samples-1)/samples
Avg = 1
Var = 0
smooth = function(x, alpha) {
	diff = x-Avg
	incr = alpha*diff
	Avg <<- Avg + incr
	Var <<- (1-alpha)*(Var+diff*incr)
}

time = 20*(1:length(x))
x = rep(1,length(time)) + sin(time/1000)
sim = tibble(time,x) 
qplot(time, x, data=sim, geom="line")

saveAvg = rep(NA,length(x))
saveVar = rep(NA,length(x))
for(i in 1:length(x)) {
	smooth(x[i], alpha)
	saveAvg[i] = Avg
	saveVar[i] = Var
}

require(tidyverse)

sim = tibble(time, x, saveAvg, saveVar, cv=sqrt(saveVar)/saveAvg*100)
qplot(time, saveAvg, data=sim, geom="line") + geom_ribbon(aes(ymax=saveAvg+sqrt(saveVar), ymin=saveAvg-sqrt(saveVar)), alpha=0.1)

qplot(time, cv, data=sim, geom="line")