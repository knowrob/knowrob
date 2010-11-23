
cup(cup0).
dairyProduct(milk1).
meatProduct(ham2).

cupboard(cupboard3).
refrigerator(fridge4).

perishable(Prod) :- dairyProduct(Prod); meatProduct(Prod).

storagePlaceFor(Loc, Item) :- refrigerator(Loc), perishable(Item).
storagePlaceFor(Loc, Item) :- cupboard(Loc), cup(Item).

searchForIn(Item,Loc) :- storagePlaceFor(Loc, Item).
