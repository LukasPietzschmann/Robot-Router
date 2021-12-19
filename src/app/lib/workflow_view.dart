import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/runtime.dart';
import 'package:robot_router/workflow.dart';

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
                  direction: DismissDirection.endToStart,
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
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: <Row>[
                      Row(
                        children: <Widget>[
                          Padding(
                            padding: const EdgeInsets.only(left: 10),
                            child: Text((index + 1).toString()),
                          ),
                          Expanded(child: item)
                        ],
                      )
                    ],
                  ));
            },
            controller: _scrollController,
          )),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              ElevatedButton(
                  onPressed: () {
                    selectBlockFromList(context).then((Block? block) => {
                          setState(() {
                            _blocks.add(block!.construct());
                            _needsScroll = true;
                          })
                        });
                  },
                  child: const Text('Add Block')),
              IconButton(
                onPressed: () => Runtime.the().exec(_blocks),
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
