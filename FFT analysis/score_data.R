### libraries
require(tidyverse) 
require(magrittr)
require(ggthemes)
require(skimr)
require(ROCR)

### numbers
nArch = 3
nFreq = 16

### load data
d.o = read_csv('data_17Jun18.csv',
	col_names = c(
		"Notes","Time","Arch","Sensor",
		"peakFreq","avgPower",
		sprintf("power_%s",1:nFreq)
	))

d = d.o %>% 
	gather(Bin, Power, starts_with("power")) %>%
	separate(Bin, c("drop","Bin")) %>%
	select(- drop) %>%
	mutate(
		Bin = as.integer(Bin),
		Arch = factor(Arch, levels=c(0,1,2), labels=c("A0","A1","A2")),
		Sensor = factor(Sensor, levels=0:5, labels=sprintf("S%d",0:5))
	) %>%
	nest(-Notes, -Time, .key="fulldata") %>%
	mutate(
		reddata = map(fulldata, function(df) { select(df, -Bin, -Power) %>% distinct } )
	) %T>% print
	
### what's when?
d %>% group_by(Notes) %>% skim(Time)	
foo = d %>% filter(between(Time, 63746, 80075)) %>% sample_n(1) %$% fulldata[[1]]

### try working with peakFreq
dpf = d %>%
	mutate(
		corr = map(reddata, function(df) {
			m = matrix(df$peakFreq, ncol=3) 
			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		})
	) %>%
	unnest(corr, .drop=T) %T>% print
	
dpf %>%
	gather(Which, Corr, starts_with("C")) %>%
	ggplot() +
	aes(x=Which, y=Corr) +
	geom_boxplot() +
	facet_wrap(~Notes) +
	coord_cartesian(ylim=c(-1,1))
	
dpf %>%
	gather(Which, Corr, starts_with("C")) %>%
	ggplot() +
	aes(x=Time, color=Which, y=Corr) +
	geom_line() +
	geom_point() +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(-1,1)) +
	scale_color_colorblind()
	
exp_smooth = function(x, alpha=0.1) {
        # Performs exponential smoothing by taking a weighted average of the
        # current and previous data points
        #
        # x     : numeric vector
        # alpha : number between 0 and 1, weight assigned to current data value
        #
        # Returns a numeric vector of the same length as x and values as the 
        #       weighted averages of the (current, previous) consecutive pairs
        s = numeric(length(x) + 1) # make s 1 cell longer than x
        for (i in seq_along(s)) {
                if (i == 1) { # set the initial value of s the same as that of x
                        s[i] = x[i]
                } else {
                        # weight current value with alpha and previous value 
                        # with 1-alpha, and sum
                        s[i] = alpha * x[i - 1] + (1 - alpha) * s[i - 1]
                }
        }
        s[-1] # drop the 1st element in s because it's extra
}

exp_smooth(foo)
	
dpf %>%
	gather(Which, Corr, starts_with("C")) %>%
	group_by(Which) %>%
	mutate(sCorr = exp_smooth(Corr, alpha=0.1)) %>%
	ggplot() +
	aes(x=Which) +
	geom_boxplot(aes(y=Corr, fill="Raw")) +
	geom_boxplot(aes(y=sCorr, fill="Smoothed")) +
	facet_wrap(~Notes) +
	coord_cartesian(ylim=c(-1,1))
	
dpf %>%
	gather(Which, Corr, starts_with("C")) %>%
	group_by(Which) %>%
	mutate(sCorr = exp_smooth(Corr, alpha=0.5)) %>%
	ggplot() +
	aes(x=Time, color=Which) +
	geom_line(aes(y=sCorr)) +
	geom_point(aes(y=Corr)) +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(-1,1)) +
	scale_color_colorblind()
	

d %>% group_by(Notes) %>% skim(Time)	
foo = d %>% filter(between(Time, 63746, 80075)) %>% slice(4) %$% fulldata[[1]]
foo %<>%
	mutate(
		Freq = (as.numeric(Bin)+0)*(1000/10)/2^7
	)
foo %>% 
	ggplot() + aes(x=Freq, y=Power) + geom_point() +
	facet_grid(Arch~Sensor) +
	facet_wrap(~Arch)

foo %>% 
	group_by(Arch, Freq) %>% 
	summarise(Power=sum(Power)) %>%
	ggplot() + aes(x=Freq, y=Power) + geom_point() +
	scale_y_log10() +
	facet_wrap(~Arch)

### how about summing up sensor frequency?
dpwr = d %>% 
	mutate(
		corr = map(fulldata, function(df) {
			m = df %>%
				group_by(Arch, Bin) %>%
				summarise(Power=log(sum(Power))) %$%
				matrix(Power, ncol=3)
				
			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		})
	) %>%
	unnest(corr, .drop=T) %T>% print
	
dpwr %>%
	gather(Which, Corr, starts_with("C")) %>%
	group_by(Which) %>%
	mutate(sCorr = exp_smooth(Corr, alpha=0.1)) %>%
	ggplot() +
	aes(x=Which) +
	geom_boxplot(aes(y=Corr, fill="Raw")) +
	geom_boxplot(aes(y=sCorr, fill="Smoothed")) +
	facet_wrap(~Notes) +
	coord_cartesian(ylim=c(0,1)) +
	scale_fill_colorblind()
	
dpwr %>%
	gather(Which, Corr, starts_with("C")) %>%
	group_by(Which) %>%
	mutate(sCorr = exp_smooth(Corr, alpha=0.5)) %>%
	ggplot() +
	aes(x=Time, color=Which) +
	geom_line(aes(y=sCorr)) +
	geom_point(aes(y=Corr)) +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(0,1)) +
	scale_color_colorblind()

	
### ok, reasonable.  
# define what the "correct"/true classification is
correct = d %>% select(Notes) %>% distinct %>%
	mutate(
		C01=c(F,F,F,F,T,T),
		C12=c(F,F,T,F,F,T),
		C20=c(F,F,F,T,F,T)
	) %T>% print
dummy = tibble(C01=F, C12=F, C20=F)
drop.first.last=3
r = d %>%
	group_by(Notes) %>%
	mutate(
		good=between(1:n(),drop.first.last,n()-drop.first.last),
		good=ifelse(Notes=="idle", T, good)
	) %>%
	ungroup %>%
	left_join(nest(correct, -Notes, .key="truth")) %>%
	mutate( truth=ifelse(good, truth, list(dummy)) ) %T>% print
	
# run the two algorithms
r %<>%
	mutate(
		pwr = map(fulldata, function(df) {
			m = df %>%
				group_by(Arch, Bin) %>%
				summarise(Power=log(sum(Power))) %$%
				matrix(Power, ncol=3)
				
			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		}),
		pwr2 = map(fulldata, function(df) {
			m = df %>%
				group_by(Arch, Bin) %>%
				summarise(Power=sum(Power)) %$%
				matrix(Power, ncol=3)
				
			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		}),
		pfreq = map(reddata, function(df) {
			m = matrix(df$peakFreq, ncol=3) 
			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		})
	) 	
tr = r %>% unnest(truth, .drop=T) %>% gather(Coord, Truth, 4:6) 
pwr = r %>% unnest(pwr, .drop=T) %>% gather(Coord, Power, 4:6) 
pwr2 = r %>% unnest(pwr2, .drop=T) %>% gather(Coord, Power2, 4:6) 
freq = r %>% unnest(pfreq, .drop=T) %>% gather(Coord, Freq, 4:6) 
res = left_join(tr, pwr) %>% left_join(freq) %>% left_join(pwr2)

fit.pwr=glm(Truth~Power+Coord+0, family="binomial", data=res)
fit.pwr2=glm(Truth~Power2+Coord+0, family="binomial", data=res)
fit.freq=glm(Truth~Freq+Coord+0, family="binomial", data=res)
AIC(fit.pwr, fit.freq, fit.pwr2)
summary(fit.pwr)

res %<>% mutate(Pred = predict(fit.pwr,type="response"))
performance(prediction(res$Pred, res$Truth), "sens", "spec") %>% plot

ggplot(res) +
	aes(x=Truth, y=Pred, fill=Coord, alpha=good) +
	geom_boxplot() +
	facet_wrap(~Notes) +
	scale_fill_colorblind()
	
### see: https://en.wikipedia.org/wiki/Pearson_correlation_coefficient#Mathematical_properties

# This formula suggests a convenient single-pass algorithm for calculating sample correlations, but, depending on the numbers involved, it can sometimes be numerically unstable.
	
	

	
