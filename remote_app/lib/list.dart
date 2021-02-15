import 'package:flutter/material.dart';


class ListRoute extends StatefulWidget {
  ListRoute({Key key}) : super(key: key);
  @override
  ListRoute createState() => ListRouteState();
}

class ListRouteState extends State<ListRoute> {
  int _selectedIndex = 0;
  static const TextStyle optionStyle =
  TextStyle(fontSize: 30, fontWeight: FontWeight.bold);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('BottomNavigationBar Sample'),
      ),
      body: Center(
        child: Text("1"),
      ),
    );
  }
}
