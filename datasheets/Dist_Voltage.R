# libraries
require(tidyverse)
require(magrittr)

# from the datasheet
d = tribble(
	~Dist,~Volt,
	
	0,0.0326087,
	10.0216,2.31522,
	15.5172,2.74845,
	20.3664,2.53416,
	30.0647,1.99845,
	40.4095,1.53727,
	50.1078,1.24379,
	60.1293,1.0528,
	70.4741,0.903727,
	80.1724,0.815217,
	90.5172,0.731366,
	100.539,0.666149,
	110.237,0.60559,
	120.259,0.568323,
	129.957,0.526398,
	139.978,0.493789,
	149.677,0.465839
)

# plot
qplot(Volt, Dist, data=d)

# accept that we can't easily figure out < 15 cm points
d %<>% mutate(use = ifelse(Dist>15, T,F))
duse = d %>% filter(use)

qplot(Volt, Dist, data=duse)

# can we easily linearlize this relationship?
qplot(1/Volt, Dist, data=duse) + geom_smooth(method="lm")
# yep

# get the relationship
fit = glm(Dist~I(1/Volt), data=duse)
# and it's valid range
1/range(1/(duse$Volt))
#Call:
#glm(formula = Dist ~ I(1/Volt), data = duse)

#Deviance Residuals: 
#    Min       1Q   Median       3Q      Max  
#-2.1212  -0.6223  -0.1436   0.5832   2.4278  

#Coefficients:
#            Estimate Std. Error t value Pr(>|t|)    
#(Intercept)  -9.0036     0.8012  -11.24 4.57e-08 ***
#I(1/Volt)    73.2242     0.5917  123.76  < 2e-16 ***
#---
#Signif. codes:  0 ‘***’ 0.001 ‘**’ 0.01 ‘*’ 0.05 ‘.’ 0.1 ‘ ’ 1
#
#(Dispersion parameter for gaussian family taken to be 1.772152)
#
#    Null deviance: 27165.062  on 14  degrees of freedom
#Residual deviance:    23.038  on 13  degrees of freedom
#AIC: 55.005
#
#Number of Fisher Scoring iterations: 2

Distance = function(Voltage) {
	Voltage[Voltage<0.4] = 0.4 # edge case, and protect from div0
	return( 73.2242*(1/Voltage) - 9.0036 )
}

# run a simulation to confirm
dsim = data_frame(
	Volt=seq(from=0.2,to=3,len=100),
	Dist=Distance(Volt)
)
qplot(Volt, Dist, data=duse) + geom_line(data=dsim)
# excellent
summary(fit)

# note that the range is [0.4-2.75V] with a supplied 5V.
# so, an ADC with Vcc=3.3V should use a 1x gain.
# but, an ADC with Vcc=5.0V using a 2x gain could see 5.5V, which is more than 0.3V over Vcc.  BAD NEWS.

