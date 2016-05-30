function [ score ] = GetCBscore( Env , x )

Wp = mvnrnd(Env.W(:),Env.Sigma);
Env.Wp = reshape(Wp,Env.Adim,Env.Wdim);

for i = 1:length(Wp(:))
    
score(i) =  ( Wp(i)-Env.W(i) );%/Env.Sigma(i);

end

score = reshape(score,Env.Adim,Env.Wdim);