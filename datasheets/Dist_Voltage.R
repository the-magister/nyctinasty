# libraries
require(tidyverse)
require(magrittr)

# from the datasheet for GP2Y0A02YK0F

d = tribble(
	~Dist,~Volt,
	
151.948,	2.76803,
201.299,	2.52545,
297.403,	1.98601,
403.896,	1.52617,
500,	1.23281,
598.701,	1.04801,
700,	0.903016,
798.701,	0.808688,
898.701,	0.714358,
997.403,	0.648981,
1097.4,	0.590839,
1201.3,	0.547162,
1302.6,	0.517967,
1401.3,	0.488779,
1500,	0.45959

) %>%
mutate(
	# assuming we have an external ADC reference of 2.75V
	Reading = Volt/2.75 * 1023,
	Reading = ifelse(Reading>1023, 1023, Reading)
)

# plot
qplot(Reading, Dist, data=d)

# accept that we can't easily figure out < 15 cm points
duse = d 

qplot(Reading, Dist, data=duse)

# can we easily linearlize this relationship?
qplot(1/Reading, Dist, data=duse) + geom_smooth(method="lm")
qplot(log(Reading), Dist, data=duse) + geom_smooth(method="lm")
# yep

# get the relationship
fit = glm(Dist~I(1/Reading), data=duse)
# and it's valid range
summary(fit)
1/range(1/(duse$Reading))
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

Distance = function(Reading) {
	# don't believe anything greater than 3m
	Reading = ifelse(Reading<85, 85, Reading)
	return( 266370.97*(1/Reading) -86.48 )
}

# run a simulation to confirm
dsim = data_frame(
	Reading=seq(from=50,to=1023),
	Dist=Distance(Reading)
)
qplot(Reading, Dist, data=duse) + geom_line(data=dsim)
# excellent
summary(fit)

# note that the range is [0.4-2.75V] with a supplied 5V.
# so, an ADC with Vcc=3.3V should use a 1x gain.
# but, an ADC with Vcc=5.0V using a 2x gain could see 5.5V, which is more than 0.3V over Vcc.  BAD NEWS.

