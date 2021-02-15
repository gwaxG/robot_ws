import 'package:flutter/material.dart';

class HistoryRoute extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("History"),
      ),
      body: Center(
        child: Text('List of done tasks!'),
      ),
    );
  }
}