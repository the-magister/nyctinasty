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
d.o = read_csv('data_23Jun18.csv',
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
	nest(-Notes, .key="fulldata") %>%
	mutate(
		reddata = map(fulldata, function(df) { select(df, -Bin, -Power) %>% distinct } )
	) %T>% print

### export Notes and manually mark up
Truth = d %>%
	select(Notes) %>%
	distinct %>%
	mutate(
		# drop these data?
		drop = F, 
		# are there humans in the arches?
		P0 = F, P1 = F, P2 = F,
		# are the humans coordinated?
		C01 = F, C12 = F, C20 = F
	) %T>% print %>%
	write_csv("truth_table_23Jun18.csv")

### must hand-edit

Truth = read_csv("truth_table_23Jun18 - edited.csv") %>%
	filter( !drop ) %>% # jettison unwanted data now.
	select( - drop ) %>% 
	nest( - Notes, .key="truth") %T>% print
	
dat = Truth %>% left_join(d)

### data load complete


### can I get P0-2 with avgPower?
avgP = dat %>% 
	unnest(truth) %>%
	unnest(reddata ) %T>% print

fit.aP.0 = glm(P0~avgPower+0, family="binomial", data=filter(avgP, Arch=="A0")) 
summary(fit.aP.0)
fit.aP.S.0 = glm(P0~Sensor*avgPower+0, family="binomial", data=filter(avgP, Arch=="A0")) 
summary(fit.aP.S.0)

avgP.r = avgP %>%
	group_by_if( is.logical ) %>%
	group_by( Notes, Time, Arch, add=T ) %>%
	summarise( 
		mean_peakFreq = mean(peakFreq),
		median_peakFreq = median(peakFreq),
		sd_peakFreq = sd(peakFreq),
		mean_avgPower = mean(avgPower),
		median_avgPower = median(avgPower),
		sd_avgPower = sd(avgPower)
	) %>% 
	ungroup %T>% print

avgP.r.A0 = filter(avgP.r, Arch=="A0") %>%
	gather(End, Value, contains("peakFreq"), contains("avgPower"))
ggplot(avgP.r.A0) +
	aes(x=P0, y=Value) +
	geom_boxplot() +
	facet_wrap(~End, scales="free_y")

# peakFreq is of no value.

avgP.r = avgP %>%
	select( - peakFreq ) %>%
	nest( Sensor, avgPower ) %>%
	mutate( 
		mean.aP = map_dbl(data, function(df) { mean(df$avgPower) } ),
		sd.aP = map_dbl(data, function(df) { sd(df$avgPower) } ),
		cv.aP = sd.aP/mean.aP*100
	) %>%
	select( - data ) %>%
	nest( P0, P1, P2, C01, C12, C20 ) %>%
	mutate(
		Player = map2_lgl(Arch, data, function(a, df) {
			if( a=="A0" ) return( df$P0[1] )
			if( a=="A1" ) return( df$P1[1] )
			if( a=="A2" ) return( df$P2[1] )
		} )
	) %>%
	select( - data ) 
	
ggplot(avgP.r) +
	aes(x=Player, y=mean.aP ) +
	geom_boxplot() +
	facet_wrap(~Arch)
ggplot(avgP.r) +
	aes(x=Player, y=sd.aP ) +
	geom_boxplot() +
	facet_wrap(~Arch)
ggplot(avgP.r) +
	aes(x=Player, y=cv.aP ) +
	geom_boxplot() +
	facet_wrap(~Arch)
	
classify = function(par, data) {
#	browser()
	par=as.list(par)
	glm(Player~factor(mean.aP>par$mean.th)+factor(sd.aP>par$sd.th)+0, family="binomial", data=data) %>%
	AIC
}
classify(list(mean.th=7000,sd.th=5000),filter(avgP.r, Arch=="A0"))
A0.th = optim(par=list(mean.th=7000, sd.th=5000), classify, data=filter(avgP.r, Arch=="A0")) %T>% print
A1.th = optim(par=list(mean.th=7000, sd.th=5000), classify, data=filter(avgP.r, Arch=="A1")) %T>% print
A2.th = optim(par=list(mean.th=6000, sd.th=3000), classify, data=filter(avgP.r, Arch=="A2")) %T>% print
all.th = optim(par=list(mean.th=6000, sd.th=3000), classify, data=avgP.r) %T>% print

A0.th$par
A1.th$par
A2.th$par
all.th$par
# mean.th    sd.th 
# 6515.625 2442.188 

ggplot(avgP.r) +
	aes(x=Player, y=mean.aP ) +
	geom_boxplot() +
	facet_wrap(~Arch) +
	geom_hline(yintercept=as.list(all.th$par)$mean.th, color="blue", linetype="dashed")

ggplot(avgP.r) +
	aes(x=Player, y=sd.aP ) +
	geom_boxplot() +
	facet_wrap(~Arch) +
	geom_hline(yintercept=as.list(all.th$par)$sd.th, color="blue", linetype="dashed")

# sd doesn't add anything.
classify = function(th, data) {
	glm(Player~factor(mean.aP>th)+0, family="binomial", data=data) %>%
	AIC
}
all.th = optimize(classify, range(avgP.r$mean.aP), data=avgP.r)

mean.th = all.th$minimum
print(mean.th)
# 6528.066
mean.th = 6528.066

ggplot(avgP.r) +
	aes(x=Notes, y=mean.aP) +
#	scale_y_log10() +
	geom_violin() +
	facet_wrap(~Player+Arch) +
	geom_hline(yintercept=mean.th, color="blue", linetype="dashed") +
	coord_flip()
	
# implement the algorithm
isPlayer = function(avgPower) {
	mean(avgPower) > mean.th
}

check.isPlayer = dat %>%
	select( - fulldata ) %>% 
	mutate(
		pred = map(reddata, function(df) {
			df %>%
				group_by(Time, Arch) %>%
				summarise( is.Player = isPlayer(avgPower) ) %>%
				spread(Arch, is.Player)
		})
	) %>%
	unnest( truth ) %>%
	unnest( pred ) %T>% print
qplot(Time, P0, data=check.isPlayer, geom="point") +
	geom_line(aes(y=A0)) +
	facet_wrap(~Notes, scales="free_x")
qplot(Time, P1, data=check.isPlayer, geom="point") +
	geom_line(aes(y=A1)) +
	facet_wrap(~Notes, scales="free_x")
qplot(Time, P2, data=check.isPlayer, geom="point") +
	geom_line(aes(y=A2)) +
	facet_wrap(~Notes, scales="free_x")

### coordination
	
### need to take care to construct a training set where arches that
# don't have players in them are filtered out.  avgPower algorithm would 
# filter those out.

keep = dat %>% 
	unnest(truth) %>%
	filter( (P0&P1) | (P1&P2) | (P2&P0) )  # toss cases where we don't have 2+ 

keep %>% print

						m = df %>%
							select(Arch, Sensor, avgPower) %>%
							distinct %$% 
							matrix(avgPower, ncol=3)					 
				
						m = df %>%
							select(Arch, Sensor, peakFreq) %>%
							distinct %$% 
							matrix(peakFreq, ncol=3)

						m = df %>%
							group_by(Arch, Bin) %>%
							summarise( Power=mean(Power) ) %$%
							matrix(Power, ncol=3)

						data.frame(
							pC01=-9,
							pC12=-9,
							pC20=-9
						) %>% return
							
						data.frame(
							pC01=cor(m[,1],m[,2]),
							pC12=cor(m[,2],m[,3]),
							pC20=cor(m[,3],m[,1])
						) %>% return
						
						df %>%
							select( - peakFreq, - avgPower ) %>%
							nest( - Bin ) %>%
							mutate( 
								corr = map(data, function(df) {
									m = df %$% matrix(Power, ncol=3)
									data.frame(
										pC01=cor(m[,1],m[,2]),
										pC12=cor(m[,2],m[,3]),
										pC20=cor(m[,3],m[,1])
									)
								})
							) %>%
							unnest( corr ) %>%
							summarise(
								pC01 = diff(range(pC01))-1,
								pC12 = diff(range(pC12))-1,
								pC20 = diff(range(pC20))-1
							)

						# this is strangely good:
						m = df %>%
							group_by(Arch, Sensor) %>%
							summarise( Power=mean(Power) ) %$%
							matrix(Power, ncol=3)
	
						m = df %>%
							group_by(Arch, Sensor) %>%
							summarise( Power=max(Power) ) %$%
							matrix(Power, ncol=3)
						data.frame(
							pC01=cor(m[,1],m[,2]),
							pC12=cor(m[,2],m[,3]),
							pC20=cor(m[,3],m[,1])
						) %>% return							

						df %>%
							select( - peakFreq, - avgPower ) %>%
							nest( - Bin ) %>%
							mutate( 
								corr = map(data, function(df) {
									m = df %$% matrix(Power, ncol=3)
									data.frame(
										pC01=cor(m[,1],m[,2]),
										pC12=cor(m[,2],m[,3]),
										pC20=cor(m[,3],m[,1])
									)
								})
							) %>%
							unnest( corr ) %>%
							summarise(
#								pC01 = diff(range(pC01))-1,
#								pC12 = diff(range(pC12))-1,
#								pC20 = diff(range(pC20))-1
#								pC01 = max(pC01),
#								pC12 = max(pC12),
#								pC20 = max(pC20)
#								pC01 = mean(pC01),
#								pC12 = mean(pC12),
#								pC20 = mean(pC20)
								pC01 = max(pC01^2),
								pC12 = max(pC12^2),
								pC20 = max(pC20^2)
							)						
	
						# this is the best so far.
						m = df %>%
							select( - peakFreq, - avgPower ) %>% 
							spread(Arch, Power)
		
						data.frame(
							pC01=cor(m$A0, m$A1),
							pC12=cor(m$A1, m$A2),
							pC20=cor(m$A2, m$A0)
						) %>% return							

	
rm(coord)
coord = keep %>%
	mutate(
		pwr = map(fulldata, function(df) {
#			browser()
			df %>%
				nest( - Time ) %>%
				mutate( 
					correl = map(data, function(df) {
#						browser()

						# this is the best so far.
						m = df %>% 
							group_by(Arch, Bin) %>% 
							summarise(Power=mean(Power)) %>% 
							spread(Arch, Power) #%>%
						#	slice(2:n()) # drop bin 1?
						
						foo = cov.wt(m%>%select(-Bin), wt=m$Bin, cor=T)
						
						return( data.frame(
							pC01=foo$cor[1,2],
							pC12=foo$cor[2,3],
							pC20=foo$cor[1,3]
						))
						
						m %>% summarise(
							pC01=which.max(A0)-which.max(A1),
							pC12=which.max(A1)-which.max(A2),
							pC20=which.max(A2)-which.max(A0)
						) %>% return
						
					})
				) %>%
				unnest( correl, .drop=T )
		})
	) 	
pwr = coord %>% 
	unnest(pwr, .drop=T) %>%
	mutate(
		# again, we can ignore cases where we don't have a player
		pC01 = ifelse(P0&P1, pC01, NA),
		pC12 = ifelse(P1&P2, pC12, NA),
		pC20 = ifelse(P2&P0, pC20, NA)
	) %T>% print
qplot(Time, pC01, color=C01, data=pwr, geom="line") +
	scale_color_colorblind() + geom_point() +
	facet_wrap(~Notes, scales="free_x") 
last_plot() + aes(y=pC12, color=C12)
last_plot() + aes(y=pC20, color=C20)
	
qplot(Time, as.numeric(C01), data=pwr, geom="point") +
	geom_line(aes(y=pC01)) +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(-1,1))
qplot(Time, as.numeric(C12), data=pwr, geom="point") +
	geom_line(aes(y=pC12)) +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(-1,1))
qplot(Time, as.numeric(C20), data=pwr, geom="point") +
	geom_line(aes(y=pC20)) +
	facet_wrap(~Notes, scales="free_x") +
	coord_cartesian(ylim=c(-1,1))

	

	ggplot(res) +
	aes(x=Notes, y=Power^2) +
#	scale_y_log10() +
	geom_violin() +
	facet_wrap(~Truth+Coord) +
	geom_hline(yintercept=0.75, color="blue", linetype="dashed") +
	coord_flip()

tr.k = keep %>% left_join(tr) %T>% print
res.k = left_join(tr.k, pwr)

last_plot() %+% res.k

qplot(Time, as.numeric(Truth), color=Coord, data=res.k, geom="point") +
	geom_line(aes(y=Power)) +
	facet_wrap(~Notes, scales="free_x") +
	scale_color_colorblind()

# nope, no good.

### try another approach?
cor.dat = dat %>% unnest(truth) %>%
	filter( (P0&P1) | (P1&P2) | (P2&P0) ) %>% # toss cases where we don't have 2+ players
	unnest( fulldata ) %T>% print
cor.red.dat = cor.dat %>%
	select( - Bin, - Power ) %>%
	distinct %T>% print
tree(C01~Arch+Sensor+peakFreq+avgPower, data=filter(cor.red.dat, P0, P1, Arch=="A0"|Arch=="A1"))

# suggests that peakFreq might be a useful classifier?

coord = cor.red.dat %>%
	nest( Arch, Sensor, peakFreq, avgPower ) %>%
	mutate(
		pf = map(data, function(df) {
			m = with(df, matrix(peakFreq, ncol=3))

			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		}),
		ap = map(data, function(df) {
			m = with(df, matrix(avgPower, ncol=3))

			data.frame(
				C01=cor(m[,1],m[,2]),
				C12=cor(m[,2],m[,3]),
				C20=cor(m[,3],m[,1])
			)
		}),		
	) %>%
	unnest(pf, ap, .sep="_", .drop=T) %T>% print

glm(C01~pf_C01+ap_C01+0, data=filter(coord, P1&P0), family="binomial")
qplot(C01, ap_C01, fill=Notes, data=filter(coord, P0&P1), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)
qplot(C12, ap_C12, fill=Notes, data=filter(coord, P1&P2), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)
qplot(C20, ap_C20, fill=Notes, data=filter(coord, P2&P0), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)

qplot(C01, pf_C01, fill=Notes, data=filter(coord, P0&P1), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)
qplot(C12, pf_C12, fill=Notes, data=filter(coord, P1&P2), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)
qplot(C20, pf_C20, fill=Notes, data=filter(coord, P2&P0), geom="boxplot") + scale_fill_colorblind() + coord_cartesian(y=c(-1,1)) + geom_hline(yintercept=0)



qplot(C01, pf_C01, fill=Notes, data=filter(coord, P1&P0), geom="boxplot") + scale_fill_colorblind()
	
# Hmm, this can't possibly work, because we're correlating across _sensors_,
# so that's looking for the same _gesture_, which is not what we want.
	
	
	
	
	
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
	
	

	

	