import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/runtime.dart';
import 'package:robot_router/workflow.dart';

import 'settings_view.dart';

class WorkflowView extends StatefulWidget {
  const WorkflowView({required this.workflow, Key? key}) : super(key: key);

  final Workflow workflow;

  @override
  _WorkflowViewState createState() => _WorkflowViewState();
}

class _WorkflowViewState extends State<WorkflowView> {
  List<BlockView> _blocks = <BlockView>[];
  final ScrollController _scrollController = ScrollController();
  bool _needsScroll = false;
  bool _isRunning = false;

  @override
  Widget build(BuildContext context) {
    if (_needsScroll) {
      _scrollController.animateTo(
        _scrollController.position.maxScrollExtent,
        curve: Curves.easeOut,
        duration: const Duration(milliseconds: 500),
      );
      _needsScroll = false;
    }
    return Scaffold(
      appBar: AppBar(title: Text(widget.workflow.name)),
      body: Column(
        children: <Widget>[
          Expanded(
            child: ListView.builder(
              itemCount: _blocks.length,
              itemBuilder: (BuildContext context, int index) {
                final BlockView item = _blocks[index];
                return Dismissible(
                    key: ObjectKey(item),
                    direction: _isRunning
                        ? DismissDirection.none
                        : DismissDirection.endToStart,
                    background: Padding(
                      padding: const EdgeInsets.all(8),
                      child: Card(
                          color: Colors.red,
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.end,
                            children: <Widget>[
                              Text('Delete ${item.block.name}',
                                  style: const TextStyle(color: Colors.white)),
                              const Icon(Icons.delete_forever_rounded,
                                  color: Colors.white)
                            ],
                          )),
                    ),
                    onDismissed: (DismissDirection direction) {
                      setState(() {
                        _blocks.removeAt(index);
                      });
                    },
                    confirmDismiss: (DismissDirection direction) async {
                      return await showDialog(
                        context: context,
                        builder: (BuildContext context) {
                          return AlertDialog(
                            title: const Text('Confirm'),
                            content: const Text(
                                'Do you really want to delete this Block?'),
                            actions: <Widget>[
                              TextButton(
                                  onPressed: () => Navigator.pop(context, true),
                                  child: const Text(
                                    'Yeah sure',
                                    style: TextStyle(color: Colors.red),
                                  )),
                              TextButton(
                                onPressed: () => Navigator.pop(context, false),
                                child: const Text('Nope'),
                              ),
                            ],
                          );
                        },
                      );
                    },
                    child: Row(
                      children: <Widget>[
                        Padding(
                          padding: const EdgeInsets.only(left: 10),
                          child: Text((index + 1).toString()),
                        ),
                        Flexible(child: item, fit: FlexFit.loose)
                      ],
                    ));
              },
              controller: _scrollController,
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              ElevatedButton(
                  onPressed: () {
                    if (_isRunning) {
                      Navigator.push(
                          context,
                          MaterialPageRoute(
                              builder: (BuildContext context) => LogsView()));
                    } else {
                      selectBlockFromList(context).then((Block? block) => {
                            block != null
                                ? setState(() {
                                    _blocks.add(block.construct());
                                    _needsScroll = true;
                                  })
                                : ''
                          });
                    }
                  },
                  child: Text(_isRunning ? 'Show logs' : 'Add Block')),
              IconButton(
                onPressed: () async {
                  setState(() {
                    _isRunning = true;
                  });
                  await Runtime.the().exec(
                      _blocks.map((BlockView block) => block.block).toList());
                  setState(() {
                    _isRunning = false;
                  });
                },
                icon: const Icon(Icons.send),
                color: Colors.indigo,
              )
            ],
          )
        ],
      ),
    );
  }
}
