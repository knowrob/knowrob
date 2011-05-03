This module calls the data base "Pancake", which is located inside the folder /database.

Before running this module, please make sure that you have properly configure the ODBC for the SQL computables by following the next steps:

1. Install the following packages: mysql-server, phpmyadmin, libmyodbc, unixodbc-bin, unixodbc-dev

2. Set up an user account: 
	$> mysql -u root -p
	mysql> CREATE USER newuser IDENTIFIED BY 'newpwd';
	mysql> GRANT ALTER,CREATE,DELETE,DROP,INSERT,SELECT,SHOW DATABASES,LOCK TABLES,UPDATE ON *.* TO newuser@localhost IDENTIFIED BY 'newpwd';
	# it might be necessary to execute the following command as root before you login as newuser:
	# mysql> flush privileges;
	mysql> exit
	bye

	# test:
	mysql -u newuser -pnewpass

3. Test the phpmyadmin web interface at http://localhost/phpmyadmin

4. Add the desired data base in the user account:
	mysql -u newuser -pnewpwd < Pancake__DB.sql

5. Configure OBDC
	$> sudo ODBCConfig
	Drivers -> Add -> Name=MySQL, 
	Driver=Driver64=/usr/lib/odbc/libmyodbc.so, 
	Setup=Setup64=/usr/lib/odbc/libodbcmyS.so
	#check that the libraries are on those paths
	confirm and exit the GUI

	$> ODBCConfig
	User DSN -> Add -> MySQl -> OK
	create one entry for every required database, e.g: Name=Database=Pancake, Server= localhost, Port= 3306, Driver= MySQL








