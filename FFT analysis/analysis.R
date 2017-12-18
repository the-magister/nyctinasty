### libraries
require(tidyverse)
require(ggthemes)
require(readxl)
require(magrittr)
require(scales)

### frequencies

### load data
d = read_csv('data.csv', col_names=F) %>%
	select("Time"=1,"Sensor"=2,"Avg"=3, everything()) %>%
	mutate(Time=signif((Time-min(Time))/1000,3)) %>%
	gather(Bin, Value, starts_with("X")) %>%
	mutate(Bin=as.numeric(gsub("X","",Bin))-4)

d %<>% filter(Bin<15)	
	
### show
qplot(Time, Value, color=Bin, group=Bin, data=d, geom="line") + facet_wrap(~Sensor, ncol=1)

### 
qplot(Bin, Value/Avg, color=Sensor, group=Sensor, data=d %>% filter(Time==39), geom="line")
