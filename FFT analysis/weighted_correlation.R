require(devtools)
 devtools::install_git('https://bitbucket.org/nicholasehamilton/ggtern')

require(tidyverse) 
require(magrittr)
require(ggtern)
require(ggthemes)

set.seed(1)
n  = 5000
df = data.frame(x     = runif(n),
                y     = runif(n),
                z     = runif(n),
                value = runif(n))
 
ggtern(df,aes(x,y,z)) + 
  geom_hex_tern(bins=5,aes(value=value,fill=..count..))

# assume we have 16 frequency buckets per arch
N_B = 16
N_A = 3

# start with unimodal distributions for arches

get.correspondence = function(df) {

	df %<>% mutate(
		bucket = 1:n(),
		weight = ifelse(between(bucket, 2, 8), 1, 0),
	)

	covar = cov.wt(
	  x = df %>% select(starts_with("arch.")),
	  wt = (df %$% weight)/(df %$% weight %>% sum),
	  cor = T
	)

	corr.AB = signif(covar$cor[1,2], 3)
	corr.BC = signif(covar$cor[2,3], 3)
	corr.CA = signif(covar$cor[1,3], 3)
	
	corr.total = signif(
		sum(covar$cor[lower.tri(covar$cor)])/3,
		3
	)
	
	tit = sprintf("AB: %s BC: %s CA: %s total: %s", corr.AB, corr.BC, corr.CA, corr.total)
	
	ggplot(df %>% gather(Arch, Value, starts_with("arch."))) +
		aes(color=Arch, x=bucket, y=Value) +
		geom_point() +
		geom_line() +
		scale_color_colorblind() +
		labs(title=tit)

}

# data frame
d = tibble(
	arch.A = dnorm(bucket, mean=1, sd=0.1), # nothing going on 
	arch.B = dnorm(bucket, mean=2, sd=1),
	arch.C = dnorm(bucket, mean=4, sd=1)
)
d
d %>% get.correspondence

# data frame
d = tibble(
	arch.A = dnorm(bucket, mean=5, sd=1), # something
	arch.B = dnorm(bucket, mean=2, sd=1),
	arch.C = dnorm(bucket, mean=4, sd=1)
)
d
d %>% get.correspondence

# data frame
d = tibble(
	arch.A = dnorm(bucket, mean=5, sd=1), # something
	arch.B = dnorm(bucket, mean=6, sd=1),
	arch.C = dnorm(bucket, mean=4, sd=1)
)
d
d %>% get.correspondence

# data frame
d = tibble(
	arch.A = dnorm(bucket, mean=5, sd=1), # something
	arch.B = dnorm(bucket, mean=6, sd=1),
	arch.C = dnorm(bucket, mean=5.5, sd=1)
)
d
d %>% get.correspondence

# data frame
d = tibble(
	arch.A = dnorm(bucket, mean=5, sd=1), # something
	arch.B = dnorm(bucket, mean=6, sd=1),
	arch.C = dnorm(bucket, mean=3, sd=1)/2 + dnorm(bucket, mean=8, sd=1)/2
)
d
d %>% get.correspondence

### confirm hand-coded version
df = d

df %<>% mutate(
	bucket = 1:n(),
	weight = ifelse(between(bucket, 2, 8), 1, 0),
)

covar = cov.wt(
  x = df %>% select(starts_with("arch.")),
  wt = (df %$% weight)/(df %$% weight %>% sum),
  cor = T
)

corr.AB = signif(covar$cor[1,2], 3)
corr.BC = signif(covar$cor[2,3], 3)
corr.CA = signif(covar$cor[1,3], 3)

corr.total = signif(
	sum(covar$cor[lower.tri(covar$cor)])/3,
	3
)

# https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Sample_covariance
# Weighted samples
N_A = 3
N_B = 16
bucket = 1:N_B
weight = ifelse(between(bucket, 2, 8), 1, 0)

# data
mat.x = d %>% select(starts_with("arch.")) %>% as.matrix

# weights
vec.w = (weight/sum(weight)) %>% matrix(ncol=1)
cov.term = 0
for(i in 1:N_B) {
	cov.term = cov.term + vec.w[i]*vec.w[i]
}
cov.term = 1/(1-cov.term)
cov.term = 1

# weighted mean vector
vec.xbar = matrix(0, nrow=1, ncol=N_A)
for(j in 1:N_A) {
	for(i in 1:N_B) {
		vec.xbar[j] = vec.xbar[j] + vec.w[i]*mat.x[i,j]
	}
}
vec.xbar
covar$center

# weighted covariance matrix
mat.cov = matrix(0, nrow=N_A, ncol=N_A)
for(j in 1:N_A) {
	for( k in j:N_A ) { # take advantage of symmetry
		sum.term = 0
		for(i in 1:N_B) {			
			sum.term = sum.term + 
				vec.w[i]*(mat.x[i,j]-vec.xbar[j])*(mat.x[i,k]-vec.xbar[k])
		}	
		mat.cov[j,k] = cov.term * sum.term
	}
}
# don't actually need this. shown for completeness.
for(j in 1:N_A) {
	for( k in 1:j ) { # take advantage of symmetry
		mat.cov[j,k] = mat.cov[k,j]
	}
}
mat.cov
covar$cov

# weighted correlation matrix
Is = matrix(0, nrow=1, ncol=N_A)
for(j in 1:N_A) {
	Is[j] = 1/sqrt(mat.cov[j,j])
}
mat.cor = matrix(0, nrow=N_A, ncol=N_A)
for(j in 1:N_A) {
	for( k in j:N_A ) { # take advantage of symmetry
		if( k==j ) { mat.cor[j,k] = 1 }
		else { mat.cor[j,k] = Is[j] * mat.cov[j,k] * Is[k] }
	}
}
# don't actually need this. shown for completeness.
for(j in 1:N_A) {
	for( k in 1:j ) { # take advantage of symmetry
		mat.cor[j,k] = mat.cor[k,j]
	}
}
mat.cor
covar$cor

corr.AB = mat.cor[1,2]
corr.BC = mat.cor[2,3]
corr.CA = mat.cor[1,3]

vec.corr = c(corr.AB , corr.BC, corr.CA)
vec.corr

corr.total = (corr.AB + corr.BC + corr.CA)/3
corr.total

### can we pack this down more for a C++ routine?
# Weighted samples
N_A = 3
N_B = 16
bucket = 1:N_B
weight = ifelse(between(bucket, 2, 8), 1, 0)

### calculate once at startup or if we adjust weights
vec.w = matrix(0, nrow=N_B, ncol=1)
sum.weight = 0
cov.term = 0
for(i in 1:N_B) {
	sum.weight = sum.weight + weight[i]
}
for(i in 1:N_B) {
	vec.w[i] = weight[i]/sum.weight
	cov.term = cov.term + vec.w[i]*vec.w[i]
}
cov.term = 1/(1-cov.term)

### when we get new data
# weighted mean vector
vec.xbar = matrix(0, nrow=1, ncol=N_A)
for(j in 1:N_A) {
	for(i in 1:N_B) {
		vec.xbar[j] = vec.xbar[j] + vec.w[i]*mat.x[i,j]
	}
}
# weighted covariance matrix
mat.cov = matrix(0, nrow=N_A, ncol=N_A)
for(j in 1:N_A) {
	for( k in j:N_A ) { # take advantage of symmetry
		sum.term = 0
		for(i in 1:N_B) {			
			sum.term = sum.term + 
				vec.w[i]*(mat.x[i,j]-vec.xbar[j])*(mat.x[i,k]-vec.xbar[k])
		}	
		mat.cov[j,k] = cov.term * sum.term
	}
}

# weighted correlation matrix
mat.cor = matrix(0, nrow=N_A, ncol=N_A)
Is = matrix(0, nrow=1, ncol=N_A)
for(j in 1:N_A) {
	Is[j] = 1/sqrt(mat.cov[j,j])
}
for(j in 1:N_A) {
	for( k in j:N_A ) { # take advantage of symmetry
		if( k!=j ) mat.cor[j,k] = Is[j] * mat.cov[j,k] * Is[k] 
	}
}

# result
corr.AB = mat.cor[1,2]
corr.BC = mat.cor[2,3]
corr.CA = mat.cor[1,3]

vec.corr = c(corr.AB , corr.BC, corr.CA)
vec.corr

corr.total = (corr.AB + corr.BC + corr.CA)/3
corr.total

