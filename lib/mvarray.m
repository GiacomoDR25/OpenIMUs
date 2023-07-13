function [m,v]=mvarray(a)
%%% Calcola modulo e versore di un vettore a
%%%[m,v]=mvarray(a) modulo e versore di un vettore
%%% Attenzione:: Usare solo vettori riga

%[m,v]=mvarray(a) modulo e versore di un vettore
m = sqrt (nansum (a.^2,2));
nD = size(a,2); %nD è pari al numero di colonne di a
v = a./repmat(m,[1,nD]); %repmat replica nD volte le colonne di m

