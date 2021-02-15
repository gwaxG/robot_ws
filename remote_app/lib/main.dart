import 'package:flutter/material.dart';

/*
* https://www.appsdeveloperblog.com/bottom-navigation-bar-example-in-flutter/
* */

void main() => runApp(MyApp());
/// This Widget is the main application widget.
class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: MyNavigationBar (),
    );
  }
}
class MyNavigationBar extends StatefulWidget {
  MyNavigationBar ({Key key}) : super(key: key);
  @override
  _MyNavigationBarState createState() => _MyNavigationBarState();
}
class _MyNavigationBarState extends State<MyNavigationBar > {
  int _selectedIndex = 0;
  static const List<Widget> _options = <Widget>[
    Text('Home Screen', style: TextStyle(fontSize: 35, fontWeight: FontWeight.bold)),
    Text('Profile Screen', style: TextStyle(fontSize: 35, fontWeight: FontWeight.bold)),
    Text('Settings Screen', style: TextStyle(fontSize: 35, fontWeight: FontWeight.bold)),
  ];
  void _onItemTap(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
          title: const Text('BottomNavigationBar Example'),
          backgroundColor: Colors.teal
      ),
      body: Center(
        child: _options.elementAt(_selectedIndex),
      ),
      bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(
                icon: Icon(Icons.home),
                label: 'Home',
                backgroundColor: Colors.teal
            ),
            BottomNavigationBarItem(
                icon: Icon(Icons.person),
                label: 'Profile',
                backgroundColor: Colors.cyan
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.settings),
              label: 'Settings',
              backgroundColor: Colors.lightBlue,
            ),
          ],
          type: BottomNavigationBarType.shifting,
          currentIndex: _selectedIndex,
          selectedItemColor: Colors.white,
          unselectedItemColor: Colors.grey,
          iconSize: 40,
          onTap: _onItemTap,
          elevation: 5
      ),
    );
  }
}