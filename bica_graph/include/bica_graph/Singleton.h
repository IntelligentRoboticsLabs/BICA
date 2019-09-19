/**
 *
 * Created: Carlos Agüero (caguero@gsyc.es) 14/05/2009
 *
**/


#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <iostream>

template< class C >
class Singleton {

public:
	static C* getInstance()
	{
		if( Singleton<C>::uniqueInstance == NULL )
			Singleton<C>::uniqueInstance = new C();

		return Singleton<C>::uniqueInstance;
	}

	static void removeInstance()
	{
		if( Singleton<C>::uniqueInstance != NULL )
		{
			delete Singleton<C>::uniqueInstance;
			Singleton<C>::uniqueInstance = NULL;
		}
	}

private:
	static C *uniqueInstance;
};

// Initialize the static member CurrentInstance
template< class C >
C* Singleton<C>::uniqueInstance = NULL;

template< class C, class D >
class SingletonRef {

public:
	static C* getInstance(D& ref)
	{
		if( SingletonRef<C, D>::uniqueInstance == NULL )
			SingletonRef<C, D>::uniqueInstance = new C(ref);

		return SingletonRef<C, D>::uniqueInstance;
	}

	static void removeInstance()
	{
		if( SingletonRef<C, D>::uniqueInstance != NULL )
		{
			delete SingletonRef<C, D>::uniqueInstance;
			SingletonRef<C, D>::uniqueInstance = NULL;
		}
	}

private:
	static C *uniqueInstance;
};

// Initialize the static member CurrentInstance
template< class C, class D >
C* SingletonRef<C, D>::uniqueInstance = NULL;



#endif /* SINGLETON_H_ */
