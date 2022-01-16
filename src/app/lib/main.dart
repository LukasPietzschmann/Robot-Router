import 'package:flutter/material.dart';
import 'package:robot_router/settings_view.dart';
import 'package:robot_router/workflow.dart';
import 'workflow_provider.dart';
import 'workflow_view.dart';

void main() {
  runApp(const App());
}

class App extends StatelessWidget {
  const App({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Robot Router',
      theme: ThemeData(
        primarySwatch: Colors.indigo,
      ),
      home: const HomePage(),
    );
  }
}

class WorkflowCard extends StatelessWidget {
  const WorkflowCard({required this.workflow, Key? key}) : super(key: key);

  final Workflow workflow;

  @override
  Widget build(BuildContext context) {
    return Card(
        shadowColor: workflow.color.withAlpha(150),
        elevation: 5,
        shape: ContinuousRectangleBorder(
          borderRadius: BorderRadius.circular(50.0),
        ),
        child: InkWell(
          splashColor: workflow.color.withAlpha(50),
          onTap: () {
            Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (BuildContext context) =>
                        WorkflowView(workflow: workflow)));
          },
          child: Center(child: Text(workflow.name)),
        ));
  }
}

class WorkflowList extends StatelessWidget {
  const WorkflowList({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return GridView.count(
      crossAxisCount: 2,
      mainAxisSpacing: 10,
      crossAxisSpacing: 10,
      padding: const EdgeInsets.all(10),
      children: <WorkflowCard>[
        for (Workflow wf in workflows) WorkflowCard(workflow: wf)
      ],
    );
  }
}

class HomePage extends StatelessWidget {
  const HomePage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(title: const Text('Robot Router'), actions: <Widget>[
          IconButton(
              onPressed: () {
                Navigator.push(
                    context,
                    MaterialPageRoute(
                        builder: (BuildContext context) => SettingsView()));
              },
              icon: const Icon(Icons.settings))
        ]),
        body: Column(
          children: <Widget>[
            const Expanded(child: WorkflowList()),
            Center(
                child: ElevatedButton(
                    onPressed: () => {print('Pressed')},
                    child: const Text('Add Workflow')))
          ],
        ));
  }
}
