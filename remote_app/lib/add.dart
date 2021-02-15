import 'package:flutter/material.dart';

class AddRoute extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Add task"),
      ),
      body: Center(
        child: Text('Task description and forms'),
      ),
    );
  }
}