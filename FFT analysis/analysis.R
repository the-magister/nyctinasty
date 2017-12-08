### libraries
require(tidyverse)
require(ggthemes)
require(readxl)
require(magrittr)

### frequencies

### load data
d = read_xlsx('Data.xlsx', col_names=F) 

fps = 20
samples = 32
bin = 0:(ncol(d)-1)
freq = bin * fps / samples

### transform
d.o = d %>%
	set_colnames( freq ) %>%
	mutate( 
		n=1:n()
	) %>%
	gather(Freq, Val, - n ) %>%
	mutate( Freq = as.numeric(Freq) )
qplot(n, Val, geom="line", data=d.o) + facet_wrap(~Freq)

rand.reading = function(N) { round(runif(N,1023,1023)) }
maxCounts = 2^16
counts = 1000
readings = rand.reading(counts)
range(readings)
values=sum(readings)

tmp = bitwShiftL(values, 6)
dist = tmp/counts
bitwShiftR(dist,6)
dist